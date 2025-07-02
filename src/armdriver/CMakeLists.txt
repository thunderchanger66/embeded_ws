cmake_minimum_required(VERSION 3.8)
project(armdriver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 查找 ROS2 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(serial REQUIRED)

set(msg_files
  "msg/ArmTargetPosition.msg"
  "msg/ArmTarget.msg"
  "msg/ColorIkResult.msg"
  "msg/FourArmPosition.msg"
  "msg/PickAndPut.msg"
  "msg/PositionColor.msg"
  "msg/SixArmPosition.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

add_executable(armdriver_node
  src/armdriver.cpp
  src/quaternion.cpp
)
target_include_directories(armdriver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 将库链接到您的可执行文件
target_link_libraries(armdriver_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# 添加 C++ 标准编译选项
ament_target_dependencies(armdriver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

# 安装目标
install(TARGETS
  armdriver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
