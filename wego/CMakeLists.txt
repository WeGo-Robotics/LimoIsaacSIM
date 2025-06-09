cmake_minimum_required(VERSION 3.8)
project(wego)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_library(isaacsim_joysitck_bridge SHARED src/isaacsim_joysitck_bridge.cc)
ament_target_dependencies(isaacsim_joysitck_bridge 
      rclcpp 
      sensor_msgs
      geometry_msgs
      rclcpp_components)
target_compile_definitions(isaacsim_joysitck_bridge PRIVATE "BUILDING_DLL")

add_library(patrol_limo SHARED src/patrol_limo.cc)
ament_target_dependencies(patrol_limo
      rclcpp 
      geometry_msgs
      nav2_msgs
      rclcpp_components
      rclcpp_action)
target_compile_definitions(patrol_limo PRIVATE "BUILDING_DLL")

rclcpp_components_register_node(
  isaacsim_joysitck_bridge
  PLUGIN "wego::IsaacsimJoystickBridge"
  EXECUTABLE joystick_bridge
)

rclcpp_components_register_node(
  patrol_limo
  PLUGIN "wego::PatrolLimo"
  EXECUTABLE patrol_limo_navigation
)

install(TARGETS 
        isaacsim_joysitck_bridge
        patrol_limo
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
