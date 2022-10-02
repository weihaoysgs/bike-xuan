#include "bike_core/BikeXuanControl.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "bike_core_control_node");
  BikeXuanControl bike_xuan_core_control;
  ros::spin();
  return 0;
}