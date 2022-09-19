cmake_minimum_required(VERSION 3.5)
project(bike_core)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

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