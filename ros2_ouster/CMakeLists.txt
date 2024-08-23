cmake_minimum_required(VERSION 3.5)
project(ros2_ouster)

# Default to compiling in Release mode. If not compiled in Release mode the 
# driver WILL fail to hit its desired output publish rate.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ouster_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# Workaround for upstream JsonCpp bug:
# https://github.com/open-source-parsers/jsoncpp/issues/1356
# This checks if the JsonCpp::JsonCpp target already exists before
# trying to find the jsoncpp package again. This prevents the
# "cannot create imported target" CMake error when multiple
# dependencies try to find jsoncpp.
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp REQUIRED)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Create separate executables for the default and Tins drivers.
set(executable_name ouster_driver)
set(tins_executable_name ouster_tins_driver)

# Create a single library target to use for both default and Tins driver.
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  ouster_msgs
  tf2_geometry_msgs
  pcl_conversions
)

add_library(${library_name} SHARED
  src/driver_types.cpp
  src/ouster_driver.cpp
  src/lifecycle_interface.cpp
  src/sensor.cpp
  src/sensor_tins.cpp
  src/client/client.cpp
  src/client/lidar_scan.cpp
  src/client/netcompat.cpp
  src/client/types.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  JsonCpp::JsonCpp
  tins
  ${PCL_LIBRARIES}
)

add_executable(${executable_name}
  src/main.cpp
)
add_executable(${tins_executable_name}
  src/main_tins.cpp
)

target_link_libraries(${executable_name} ${library_name})
target_link_libraries(${tins_executable_name} ${library_name})

rclcpp_components_register_nodes(${library_name} "${PROJECT_NAME}::Driver")
rclcpp_components_register_nodes(${library_name} "${PROJECT_NAME}::TinsDriver")

set(node_plugins "${node_plugins}${PROJECT_NAME}::Driver;$<TARGET_FILE:ouster_driver>\n")
set(node_plugins "${node_plugins}${PROJECT_NAME}::TinsDriver;$<TARGET_FILE:ouster_driver>\n")

install(TARGETS ${executable_name} ${tins_executable_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()