#ifndef CANRECEIVESEND_HPP
#define CANRECEIVESEND_HPP

#include <glog/logging.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <chrono>
#include <memory>
#include <thread>

#include "bike_core/odrive_can_msg.h"

class CanSendReceive {
 public:
  CanSendReceive();
  ~CanSendReceive() = default;
  void tReceivePublishCanMsg() const;

 public:
  static int GetOneSocketCanInstance(const std::string &can_port_name);

  static int WriteDataToSocketCanDevice(const int &socket_can_fd,
                                        const canid_t &can_id,
                                        const std::array<int16_t, 4> &data = {
                                            0, 0, 0, 0});

 private:
  std::string receive_can_port_name_;
  ros::NodeHandle nh_;
  ros::Publisher pub_can_msg_;
};

#endif  // CANRECEIVESEND_HPP