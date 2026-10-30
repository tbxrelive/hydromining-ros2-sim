cmake_minimum_required(VERSION 3.8)
project(hydromining_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)
add_executable(coal_wall_scene src/coal_wall_scene.cpp)
target_compile_features(coal_wall_scene PRIVATE cxx_std_17)
target_link_libraries(coal_wall_scene PRIVATE gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 protobuf::libprotobuf)
install(TARGETS coal_wall_scene DESTINATION lib/${PROJECT_NAME})
add_executable(jet_bridge src/jet_bridge.cpp)
target_compile_features(jet_bridge PRIVATE cxx_std_17)
ament_target_dependencies(jet_bridge rclcpp ros_gz_bridge)
install(TARGETS jet_bridge DESTINATION lib/${PROJECT_NAME})
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SprayAction.srv"
  "msg/SprayStatus.msg"
  "msg/ObservationView.msg"
  "msg/CoalWallState.msg"
  "action/AssessRegion.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# 安装目录
install(DIRECTORY
  models
  urdf
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 使 launch 和 scripts 文件可执行
install(PROGRAMS
  scripts/test_motion.py
  scripts/pointcloud_capture.py
  scripts/spray_controller_node.py
  scripts/wall_perception_node.py
  test/scripts/test_observation.py
  test/scripts/verify_registration.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/gazebo_coal_wall.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/perception_observation.py DESTINATION lib/${PROJECT_NAME})

ament_package()
