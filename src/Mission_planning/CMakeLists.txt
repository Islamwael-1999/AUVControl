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


add_executable(set_waypoint src/set_waypoint.cpp )

ament_target_dependencies( set_waypoint uuv_control_msgs  rclcpp )


install(TARGETS

  set_waypoint
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
