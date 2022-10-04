#ifndef CANRECEIVESEND_HPP
#define CANRECEIVESEND_HPP

#include <glog/logging.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <thread>
#include <tuple>

#include "bike_core/odrive_can_msg.h"
#include "bike_core/odrive_motor_feedback_msg.h"

class CanSendReceive {
 public:
  CanSendReceive();
  ~CanSendReceive() = default;
  /**
   * \brief receive all odrive can message, parse and publish them to ros.
   */
  void tReceivePublishCanMsg() const;
  /**
   * \brief the thread send the special command to odrive can interface, so that
   * can receive the motor speed and position data eg.
   */
  void tSendSpecialCommand() const;
  /**
   * \brief parser special odrive can message
   */
  void ParserSpecialCanMessage(uint32_t can_id,
                               can_frame &receive_can_frame) const;

 public:
  /**
   * \brief get one socket can send fd, if using the return fd to receive can
   * message, please use next api
   * \param can_port_name can port name, eg. "can0"、"can1"
   * \retval if success get, return int value larger 0, or return -1
   */
  static int GetOneSocketCanInstance(const std::string &can_port_name);

  /**
   * \brief this fun have some error to be solve
   */
  static int WriteDataToSocketCanDevice(const int &socket_can_fd,
                                        const canid_t &can_id,
                                        const bool is_RTR,
                                        const std::array<uint8_t, 8> &data);
  /**
   * \brief get one socket can receive fd
   * \param can_port_name can port name, eg. "can0"、"can1"
   * \retval if success get, return int value larger 0, or return -1
   */
  static int GetOneSocketCanSendInstance(const char *port_name);

 private:
  std::string receive_can_port_name_;
  ros::NodeHandle nh_;
  ros::Publisher pub_can_msg_;
  ros::Publisher pub_odrive_motor_msg_;
};

#endif  // CANRECEIVESEND_HPP