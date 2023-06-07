cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 

  "msg/Waypoint.msg" 
  "srv/GoTo.srv" 

  DEPENDENCIES 
  builtin_interfaces 
  std_msgs 
  geometry_msgs)


ament_export_dependencies(rosidl_default_runtime )
ament_package()
