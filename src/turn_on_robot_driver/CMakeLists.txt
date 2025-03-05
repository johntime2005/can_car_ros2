cmake_minimum_required(VERSION 3.5)
project(turn_on_robot_driver)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖包
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
find_package(ament_cmake_python REQUIRED)
find_package(type_description_interfaces REQUIRED)

# 链接库所在路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 设置头文件查找路径
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

# 定义 turn_on_robot_driver 可执行文件
add_executable(turn_on_robot_driver src/turn_on_robot_driver.cpp)
target_link_libraries(turn_on_robot_driver
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
ament_target_dependencies(turn_on_robot_driver
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
  type_description_interfaces
)

# 定义 keycontrol 可执行文件
add_executable(keycontrol src/wheeltec_keyboard.cpp)
ament_target_dependencies(keycontrol
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
  type_description_interfaces
)

# 添加launch可执行文件
add_executable(launch_nodes src/launch.cpp)
ament_target_dependencies(launch_nodes
  rclcpp
)

# 安装可执行文件到 lib/${PROJECT_NAME} 目录
install(TARGETS
  turn_on_robot_driver
  keycontrol
  launch_nodes
  # data_fetch
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()