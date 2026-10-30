// SPDX-License-Identifier: Apache-2.0
// Architecture reference: topguns837/gz_sim_spray_painting_plugin (Apache-2.0).
// Independently implemented: timed decay, bounded emitters, +Z jet and no paint deposition.
#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/math/Quaternion.hh>
#include <gz/sim/System.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParticleEmitter.hh>
#include <sdf/Root.hh>

namespace hydromining
{
/// Bounded display effects only. Commands contain world geometry, never erosion forces.
class WaterJetEffects final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  using Entity = gz::sim::Entity;
  using Vec = gz::math::Vector3d;
  std::mutex mutex_;
  gz::transport::Node transport_;
  gz::transport::Node::Publisher diagnostics_;
  std::vector<double> pending_;
  gz::sim::EventManager *events_{};
  Entity world_{gz::sim::kNullEntity};
  Entity model_{gz::sim::kNullEntity};
  Entity core_{gz::sim::kNullEntity};
  std::vector<Entity> emitters_;
  std::array<Entity, 3> links_{};
  std::map<std::string, Entity> namedLinks_;
  double sequence_{-1}, lastMessage_{0}, stopTime_{0}, length_{0}, rate_{0};
  double lastReport_{-1};
  bool active_{false}, draining_{false};
  unsigned counter_{0};

  static std::string Pose(const Vec &p, const Vec &direction, bool zAxis=false)
  {
    gz::math::Quaterniond q;
    q.SetFrom2Axes(zAxis ? Vec::UnitZ : Vec::UnitX, direction);
    std::ostringstream out;
    out << p << ' ' << q.Euler();
    return out.str();
  }

  void Receive(const gz::msgs::StringMsg &msg)
  {
    std::istringstream stream(msg.data());
    std::vector<double> values;
    double x;
    while (stream >> x) values.push_back(x);
    if (!stream.eof() || values.empty() ||
        !((values[0]==1 && values.size()==23) || (values[0]==2 && values.size()==29)) ||
        !std::all_of(values.begin(), values.end(), [](double v){return std::isfinite(v);}))
      return;
    if (values[2]<0 || values[2]>2 || values[16]<0 || values[16]>2 ||
        values[22]<0.001 || values[22]>0.03) return;
    for (const auto i : {15,17,18,19,20,21})
      if (values[i]!=0 && values[i]!=1) return;
    if (values.size()==29)
    {
      for (int i=23; i<26; ++i) if (values[i]<0 || values[i]>1) return;
      for (int i=26; i<29; ++i) if (values[i]<.02 || values[i]>.5) return;
    }
    const Vec d(values[6],values[7],values[8]);
    if (std::abs(d.Length()-1)>1e-4) return;
    if (values[15]>0 && std::abs(Vec(values[12],values[13],values[14]).Length()-1)>1e-4) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_=std::move(values);
  }

  void Clear(gz::sim::EntityComponentManager &ecm)
  {
    if (model_!=gz::sim::kNullEntity) ecm.RequestRemoveEntity(model_, true);
    model_=core_=gz::sim::kNullEntity;
    emitters_.clear(); namedLinks_.clear(); links_.fill(gz::sim::kNullEntity);
    active_=draining_=false; rate_=0;
  }

  void Stop(double now, gz::sim::EntityComponentManager &ecm)
  {
    if (!active_) return;
    if (core_!=gz::sim::kNullEntity) ecm.RequestRemoveEntity(core_, true);
    core_=gz::sim::kNullEntity;
    for (const auto entity : emitters_)
    {
      gz::msgs::ParticleEmitter command;
      command.mutable_emitting()->set_data(false);
      ecm.CreateComponent(entity, gz::sim::components::ParticleEmitterCmd(command));
    }
    active_=false; draining_=true; stopTime_=now;
  }

  void Start(const std::vector<double> &v, gz::sim::EntityComponentManager &ecm)
  {
    const Vec source(v[3],v[4],v[5]), d(v[6],v[7],v[8]);
    const Vec hit(v[9],v[10],v[11]), normal(v[12],v[13],v[14]);
    length_=std::clamp((hit-source).Length(), 0.02, 3.0);
    const double strength=v[16];
    const bool configured=v.size()==29;
    const double mistScale=configured?v[23]:1, splashScale=configured?v[24]:1,
                 dustScale=configured?v[25]:1;
    const std::string prefix="water_jet_"+std::to_string(++counter_);
    const std::string assets=HYDROMINING_JET_ASSETS;
    std::ostringstream xml;
    xml << "<sdf version='1.9'><model name='" << prefix << "'><static>true</static>";
    if (v[18]>0)
      xml << "<link name='core'><pose>" << Pose(source,d,true)
          << "</pose><visual name='water'><cast_shadows>false</cast_shadows><geometry><mesh><uri>"
          << assets << "/meshes/jet.obj</uri><scale>" << v[22] << ' ' << v[22] << ' ' << length_
          << "</scale></mesh></geometry><material><ambient>0.5 0.8 0.95 1</ambient>"
          << "<diffuse>0.5 0.8 0.95 1</diffuse><pbr><metal><metalness>0</metalness>"
          << "<roughness>0.2</roughness><albedo_map>" << assets
          << "/materials/textures/water.png</albedo_map></metal></pbr></material>"
          << "<transparency>0.25</transparency></visual></link>";

    // Rates * lifetimes <= 1200 live billboards at strength <= 2. No rigid drops.
    auto emitter=[&](const std::string &name, const Vec &p, const Vec &axis,
                     double rate, double life, double size, double velocity,
                     const std::string &color, double scatter, double box)
    {
      if (rate<=0) return;
      rate_+=rate;
      xml << "<link name='" << name << "'><pose>" << Pose(p,axis)
          << "</pose><particle_emitter name='" << name << "' type='box'>"
          << "<emitting>true</emitting><size>" << std::min(.04,box) << ' ' << box << ' ' << box
          << "</size><particle_size>" << size << ' ' << size << ' ' << size
          << "</particle_size><rate>" << rate << "</rate><lifetime>" << life
          << "</lifetime><min_velocity>" << velocity*.6 << "</min_velocity><max_velocity>"
          << velocity << "</max_velocity><scale_rate>0.04</scale_rate>"
          << "<color_start>" << color << "</color_start><color_end>1 1 1 0</color_end>"
          << "<color_range_image>" << assets << "/materials/textures/fade.png</color_range_image>"
          << "<particle_scatter_ratio>" << scatter << "</particle_scatter_ratio>"
          << "<material><diffuse>" << color << "</diffuse><pbr><metal><albedo_map>"
          << assets << "/materials/textures/mist.png</albedo_map></metal></pbr></material>"
          << "</particle_emitter></link>";
    };
    rate_=0;
    // Flowing textured droplets on the core. Display speed has no physical role.
    emitter("stream",source,d,v[18]*80*strength,std::min(.6,length_/3),.008,3,
            "0.8 0.95 1 0.7",.05,.005);
    if (v[15]>0)
    {
      const Vec outside=hit+normal*.08;
      emitter("mist",outside,normal,v[19]*110*strength*mistScale,2.0,.14,.08,
              "0.9 0.95 1 0.32",.45,configured?v[26]:.24);
      emitter("splash",outside,normal,v[20]*65*strength*splashScale,.6,.022,.7,
              "0.75 0.9 1 0.7",.15,configured?v[27]:.10);
      emitter("dust",outside,normal,v[21]*v[17]*35*strength*dustScale,2.5,.10,.07,
              "0.25 0.20 0.16 0.23",.25,configured?v[28]:.20);
    }
    xml << "</model></sdf>";
    sdf::Root root;
    const auto errors=root.LoadSdfString(xml.str());
    if (!errors.empty() || !root.Model()) return;
    gz::sim::SdfEntityCreator creator(ecm,*events_);
    model_=creator.CreateEntities(root.Model());
    creator.SetParent(model_, world_);
    for (const auto &name : {"core","stream","mist","splash","dust"})
    {
      const auto link=ecm.EntityByComponents(gz::sim::components::Name(name),
          gz::sim::components::ParentEntity(model_));
      if (name==std::string("core")) core_=link;
      namedLinks_[name]=link;
      if (link!=gz::sim::kNullEntity)
      {
        ecm.Each<gz::sim::components::ParticleEmitter, gz::sim::components::ParentEntity>(
          [&](const Entity &child, const auto *, const auto *parent)
          { if (parent->Data()==link) emitters_.push_back(child); return true; });
      }
    }
    active_=true; draining_=false;
  }

 public:
  void Configure(const Entity &world, const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &events) override
  {
    events_=&events;
    world_=world;
    transport_.Subscribe("/hydromining/jet/command", &WaterJetEffects::Receive, this);
    diagnostics_=transport_.Advertise<gz::msgs::StringMsg>("/hydromining/jet/diagnostics");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if (now<lastMessage_ || now<lastReport_)
    { Clear(ecm); lastMessage_=now; lastReport_=now-.2; sequence_=-1; }
    std::vector<double> v;
    { std::lock_guard<std::mutex> lock(mutex_); v.swap(pending_); }
    if (!v.empty() && v[1]>=sequence_)
    {
      lastMessage_=now;
      if (v[2]==0) { Clear(ecm); sequence_=v[1]; }
      else if (v[2]==2) Stop(now,ecm);
      else if (v[1]>sequence_ || (!active_ && !draining_))
      {
        Clear(ecm); sequence_=v[1];
        if (v[16]>0) Start(v,ecm);
      }
      if (active_ && v[2]==1)
      {
        const Vec source(v[3],v[4],v[5]), d(v[6],v[7],v[8]);
        const Vec outside=Vec(v[9],v[10],v[11])+Vec(v[12],v[13],v[14])*.08;
        for (const auto &[name,entity] : namedLinks_)
        {
          if (entity==gz::sim::kNullEntity || !ecm.HasEntity(entity)) continue;
          const bool atSource=name=="core" || name=="stream";
          gz::math::Quaterniond q;
          q.SetFrom2Axes(name=="core" ? Vec::UnitZ : Vec::UnitX,
                        atSource ? d : Vec(v[12],v[13],v[14]));
          ecm.SetComponentData<gz::sim::components::Pose>(entity,
              gz::math::Pose3d(atSource ? source : outside,q));
        }
      }
    }
    if (active_ && now-lastMessage_>1.0) Stop(now,ecm);
    if (draining_ && now-stopTime_>2.6) Clear(ecm);
    if (now-lastReport_>=.1)
    {
      gz::msgs::StringMsg msg;
      std::ostringstream out;
      out << sequence_ << ' ' << active_ << ' ' << draining_ << ' ' << emitters_.size()
          << ' ' << rate_ << ' ' << length_ << ' ' << now;
      msg.set_data(out.str()); diagnostics_.Publish(msg); lastReport_=now;
    }
  }
};
}
GZ_ADD_PLUGIN(hydromining::WaterJetEffects, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hydromining::WaterJetEffects, "hydromining::WaterJetEffects")
