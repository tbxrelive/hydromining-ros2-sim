# Water jet effects provenance

Project code and procedural assets: Apache-2.0, consistent with hydromining_sim.

Architecture references (consulted 2026-10-26):

- Gazebo Sim 8 ParticleEmitter tutorial and implementation, Open Source Robotics Foundation, Apache-2.0:
  https://gazebosim.org/api/sim/8/particle_emitter.html
  https://github.com/gazebosim/gz-sim/tree/gz-sim8
- topguns837/gz_sim_spray_painting_plugin, Apache-2.0:
  https://github.com/topguns837/gz_sim_spray_painting_plugin
  https://github.com/topguns837/gz_sim_spray_painting_plugin/blob/master/LICENSE

The jet_bridge executable also links the installed Apache-2.0 ros_gz_bridge component and uses its message converters: https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge . No third-party source files or textures are vendored. The local plugin independently implements bounded native emitters, world parenting, nozzle-axis conversion, heartbeat/decay/reset and a tapered core; it does not include paint-deposition logic, the Docker environment, or the original fixed display speed. Local jet.obj, water.png and mist.png are procedurally generated project assets.

SPH references (not linked or bundled): DualSPHysics (LGPL-2.1) and SPlisHSPlasH (MIT). Native render particles are not SPH particles and do not constitute a fluid/coal impact validation.
