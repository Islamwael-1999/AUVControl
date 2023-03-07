cmake_minimum_required(VERSION 3.5)
project(Mission_planning)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uuv_control_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)


add_executable(set_waypoint src/set_waypoint.cpp )


add_executable(set_waypoint_1 src/set_waypoint_1.cpp )

add_executable(mission_execute src/mission_execute.cpp )

add_executable(behaviour_2 src/behaviour_2.cpp )

ament_target_dependencies( set_waypoint  uuv_control_msgs tf2 gazebo_msgs behaviortree_cpp_v3 rclcpp )
ament_target_dependencies( set_waypoint_1  uuv_control_msgs tf2 gazebo_msgs behaviortree_cpp_v3 rclcpp )
ament_target_dependencies( mission_execute  uuv_control_msgs tf2 gazebo_msgs behaviortree_cpp_v3 rclcpp )
ament_target_dependencies( behaviour_2  uuv_control_msgs tf2 gazebo_msgs behaviortree_cpp_v3 rclcpp )

install(TARGETS
  behaviour_2
  set_waypoint
  set_waypoint_1
  mission_execute
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
