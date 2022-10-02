#include "bike_core/CanReceiveSend.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_send_receive_node");
  CanSendReceive can;
  ros::spin();
  return 0;
}