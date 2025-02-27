cmake_minimum_required(VERSION 3.5)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(cmd_vel src/cmd_vel.cpp)

target_link_libraries(cmd_vel
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  canbus
  ${LIBUSB_LIBRARIES}
)

ament_target_dependencies(cmd_vel 
  rclcpp 
  nav_msgs 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
)

install(TARGETS cmd_vel
  DESTINATION lib/${PROJECT_NAME})

ament_package()