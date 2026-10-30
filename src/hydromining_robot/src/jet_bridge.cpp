// SPDX-License-Identifier: Apache-2.0
// Reuse installed ros_gz_bridge conversions; explicitly order callback teardown.
#include <csignal>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void RequestStop(int) { stopping = 1; }

/// Existing upstream bridge component, with transport stopped before ROS context.
class OrderedBridge : public ros_gz_bridge::RosGzBridge
{
 public:
  void Start()
  {
    Add("/hydromining/jet/command", "std_msgs/msg/String", "gz.msgs.StringMsg",
        ros_gz_bridge::BridgeDirection::ROS_TO_GZ, false);
    Add("/hydromining/jet/diagnostics", "std_msgs/msg/String", "gz.msgs.StringMsg",
        ros_gz_bridge::BridgeDirection::GZ_TO_ROS, false);
    Add("/hydromining/jet/overview", "sensor_msgs/msg/Image", "gz.msgs.Image",
        ros_gz_bridge::BridgeDirection::GZ_TO_ROS, true);
    RCLCPP_INFO(get_logger(), "Jet bridge ready; upstream ros_gz_bridge conversions, ordered shutdown");
  }

  void Stop()
  {
    if (!gz_node_) return;
    // Prevent incoming callbacks from referring to bridge handles being deleted.
    for (const auto &topic : gz_node_->SubscribedTopics()) gz_node_->Unsubscribe(topic);
    handles_.clear();
    gz_node_.reset();
  }

 private:
  void Add(const std::string &topic, const std::string &rosType, const std::string &gzType,
           ros_gz_bridge::BridgeDirection direction, bool sensor)
  {
    ros_gz_bridge::BridgeConfig config;
    config.ros_topic_name=config.gz_topic_name=topic;
    config.ros_type_name=rosType; config.gz_type_name=gzType;
    config.direction=direction;
    config.qos_profile=sensor ? rclcpp::QoS(1).best_effort() : rclcpp::QoS(10).reliable();
    add_bridge(config);
  }
};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);
  auto node=std::make_shared<OrderedBridge>();
  node->Start();  // add_bridge uses shared_from_this(), after construction
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (!stopping && rclcpp::ok()) executor.spin_once(std::chrono::milliseconds(50));
    node->Stop();
    executor.remove_node(node);
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
