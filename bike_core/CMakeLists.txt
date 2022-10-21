cmake_minimum_required(VERSION 3.5)
project(bike_core)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf2
  tf
  tf2_ros
  tf2_geometry_msgs
)

# add opencv lib
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

# Add msg file
add_message_files(
  FILES
  remote_control_msg.msg
  odrive_can_msg.msg
  odrive_motor_feedback_msg.msg
  pid_params_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets Core SerialPort REQUIRED)
qt5_wrap_cpp(header_SRC include/bike_core/ImuCH100.hpp)

add_executable(sub_imu_test_node
test/sub_imu_test_node.cpp
src/ImuCH100.cpp
${header_SRC}
)

target_link_libraries(sub_imu_test_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::SerialPort
  glog
)

# 解析遥控器数据节点
qt5_wrap_cpp(parser_remote_data_node_header_SRC include/bike_core/RemoteControlParser.hpp)

add_executable(parser_remote_data_node
node/ParserRemoteDataNode.cpp
src/RemoteControlParser.cpp
${parser_remote_data_node_header_SRC}
)
target_link_libraries(parser_remote_data_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::SerialPort
  glog
)


# 测试 odrive can 数据接受以及解析节点
add_executable(test_can_send_receive_node
test/can_send_receive_node.cpp
src/CanReceiveSend.cpp
)
target_link_libraries(test_can_send_receive_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog
)
# BikePid test node
add_executable(bike_pid_test_node
test/bike_pid_test_node.cpp
src/BikePid.cpp
)
target_link_libraries(bike_pid_test_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog
)

# ServoControl test node
add_executable(servo_control_test_node
test/servo_control_test_node.cpp
src/ServoControl.cpp
)
target_link_libraries(servo_control_test_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog
)


# odrive can 数据接受以及解析节点
add_executable(can_send_receive_node
node/CanSendReceiveNode.cpp
src/CanReceiveSend.cpp
)
target_link_libraries(can_send_receive_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog
)

# 核心控制节点
add_executable(bike_xuan_core_control_node
node/BikeXuanCoreControlNode.cpp
src/CanReceiveSend.cpp
src/BikeXuanControl.cpp
src/RemoteControlParser.cpp
src/BikePid.cpp
src/ServoControl.cpp
src/OdriveMotorConfig.cpp
${parser_remote_data_node_header_SRC}
)
target_link_libraries(bike_xuan_core_control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::SerialPort
  glog
)