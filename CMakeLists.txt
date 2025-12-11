cmake_minimum_required(VERSION 3.8)
project(uwb_imu_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 查找Eigen3
find_package(Eigen3 REQUIRED)
# 或者使用ament_cmake的eigen3_cmake_module
find_package(eigen3_cmake_module REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Uwb.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 查找串口库 (Linux)
# 注意：Windows上可能需要不同的串口库，如Windows Serial Port
# 在Linux上使用libserial，在Windows上可能需要使用其他库
if(UNIX AND NOT APPLE)
  find_package(PkgConfig REQUIRED)
  # pkg_check_modules(LIBSERIAL REQUIRED libserial)
  set(HAVE_LIBSERIAL TRUE)
else()
  # Windows或其他平台，可能需要使用其他串口库
  # 这里暂时设置为空，如果需要在Windows上编译，请添加相应的串口库
  set(LIBSERIAL_LIBRARIES "")
  set(LIBSERIAL_INCLUDE_DIRS "")
  set(HAVE_LIBSERIAL FALSE)
  message(WARNING "libserial not found. Serial functionality may be limited.")
endif()

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

if(HAVE_LIBSERIAL)
  include_directories(${LIBSERIAL_INCLUDE_DIRS})
endif()

# 源文件
set(SOURCES
  src/eskf.cpp
  src/serial_reader.cpp
  src/trilateration.cpp
  src/uwb_location_node.cpp
)

# 头文件
set(HEADERS
  include/eskf.hpp
  include/fusion_interface.hpp
  include/initializer.hpp
  include/serial_reader.hpp
  include/trilateration.h
  include/uwb_location_node.hpp
)

# 可执行文件：主融合节点
add_executable(${PROJECT_NAME}_node src/main.cpp ${SOURCES})

# 链接库
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

# 链接自定义消息
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# 可执行文件：UWB驱动节点
add_executable(uwb_pub_node src/uwb.cpp src/serial_reader.cpp)

# 链接库
ament_target_dependencies(uwb_pub_node
  rclcpp
  geometry_msgs
  visualization_msgs
)

# 链接自定义消息
rosidl_target_interfaces(uwb_pub_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(HAVE_LIBSERIAL)
  target_link_libraries(uwb_pub_node
    ${LIBSERIAL_LIBRARIES}
  )
  target_include_directories(uwb_pub_node PUBLIC
    ${LIBSERIAL_INCLUDE_DIRS}
  )
endif()

target_include_directories(uwb_pub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node uwb_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.xml" PATTERN "*.yaml"
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# 安装rviz配置文件
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# 导出依赖
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  Eigen3
  rosidl_default_runtime
)

# 测试（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

