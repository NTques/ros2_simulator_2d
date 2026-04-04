cmake_minimum_required(VERSION 3.8)
project(simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
set(CMAKE_AUTOMOC ON)

include_directories(include)

set(SOURCES
  src/main.cpp
  src/map_loader.cpp
  src/robot.cpp
  src/obstacle_manager.cpp
  src/physics_engine.cpp
  src/simulator_node.cpp
  src/sweep_runner.cpp
  src/gui/main_window.cpp
  src/gui/map_view.cpp
  src/gui/sweep_panel.cpp
  src/gui/param_config_dialog.cpp
  # Headers with Q_OBJECT must be listed so AUTOMOC processes them
  include/simulator/gui/main_window.hpp
  include/simulator/gui/map_view.hpp
  include/simulator/gui/sweep_panel.hpp
  include/simulator/gui/param_config_dialog.hpp
  include/simulator/simulator_node.hpp
  include/simulator/sweep_runner.hpp
)

add_executable(simulator_node ${SOURCES})

target_link_libraries(simulator_node
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  yaml-cpp
)

ament_target_dependencies(simulator_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  nav2_msgs
  rclcpp_action
  rosgraph_msgs
)

install(TARGETS simulator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
