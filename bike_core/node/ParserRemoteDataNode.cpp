#include "bike_core/RemoteControlParser.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "parser_remote_data_node");
  RemoteControlDataParser parser;
  ROS_INFO("Hello");
  ros::spin();
  return 0;
}