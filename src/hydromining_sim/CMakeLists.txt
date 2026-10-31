cmake_minimum_required(VERSION 3.8)
project(hydromining_sim)
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
set(CMAKE_AUTOMOC ON)
qt5_add_resources(status_resources plugins/status.qrc)
add_library(HydrominingStatus SHARED plugins/HydrominingStatus.cc plugins/HydrominingStatus.hh ${status_resources})
target_link_libraries(HydrominingStatus PRIVATE gz-gui8::gz-gui8 gz-plugin2::register Qt5::Core Qt5::Quick Qt5::QuickControls2)
install(TARGETS HydrominingStatus DESTINATION lib)
add_library(WaterJetEffects SHARED plugins/WaterJetEffects.cc)
target_compile_features(WaterJetEffects PRIVATE cxx_std_17)
target_compile_definitions(WaterJetEffects PRIVATE
  HYDROMINING_JET_ASSETS="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/water_jet")
target_link_libraries(WaterJetEffects PRIVATE gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-plugin2::register)
install(TARGETS WaterJetEffects DESTINATION lib)

# 安装目录
install(DIRECTORY
  models
  worlds
  scripts/algorithms
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# 安装 Python 脚本（保持可执行）
install(PROGRAMS
  scripts/generate_coal_wall.py
  scripts/generate_from_preset.py
  scripts/rebuild_coal_wall.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
