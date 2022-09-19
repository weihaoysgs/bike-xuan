cmake_minimum_required(VERSION 3.0.2)
project(bike_core)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sub_imu_test_node
test/sub_imu_test_node.cpp
)


target_link_libraries(sub_imu_test_node
  ${catkin_LIBRARIES}
  glog
)