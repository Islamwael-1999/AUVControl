cmake_minimum_required(VERSION 3.5)
project(odom_to_tf2)


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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_to_tf2 src/odom_to_tf2.cpp)
ament_target_dependencies(odom_to_tf2 rclcpp nav_msgs tf2_ros)

install(TARGETS odom_to_tf2
        DESTINATION lib/${PROJECT_NAME})

ament_package()
