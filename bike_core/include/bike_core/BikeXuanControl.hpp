#ifndef BIKEXUAN_CONTROL_HPP
#define BIKEXUAN_CONTROL_HPP

#include "bike_core/CanReceiveSend.hpp"
#include "bike_core/odrive_can_msg.h"
#include "bike_core/odrive_motor_feedback_msg.h"
#include "bike_core/remote_control_msg.h"

class BikeXuanControl {
 public:
  BikeXuanControl();
  ~BikeXuanControl() = default;
  const bool ChechSubscriberMessageTimestamp() const;
 private:
  void SubOdriveCanSourceMessageCB(
      const bike_core::odrive_can_msg::ConstPtr &msg) {
    *odrive_src_can_msg_ptr_.get() = *msg;
  };
  void SubRemoteControlMessageCB(
      const bike_core::remote_control_msg::ConstPtr &msg) {
    *rc_ctrl_msg_ptr_.get() = *msg;
    LOG_IF(WARNING, 0) << "ch0: " << rc_ctrl_msg_ptr_->ch_x[0] << " "
                       << "ch1: " << rc_ctrl_msg_ptr_->ch_x[1] << " "
                       << "ch2: " << rc_ctrl_msg_ptr_->ch_x[2] << " "
                       << "ch3: " << rc_ctrl_msg_ptr_->ch_x[3] << " "
                       << "s1: " << static_cast<int>(rc_ctrl_msg_ptr_->s1)
                       << " "
                       << "s2: " << static_cast<int>(rc_ctrl_msg_ptr_->s2);
  };
  void SubOdriveMotorFeedbackParsedMessageCB(
      const bike_core::odrive_motor_feedback_msg::ConstPtr &msg) {
    *odrive_can_parsed_msg_ptr_.get() = *msg;
    LOG_IF(WARNING, 0) << "Speed: " << odrive_can_parsed_msg_ptr_->speed << " "
                       << "Position: " << odrive_can_parsed_msg_ptr_->position
                       << " "
                       << "Can ID: " << odrive_can_parsed_msg_ptr_->can_id;
  };
  void SubIMUCH100MessageCallback();

 private:
  // void timerthreadBikeCoreControl(const ros::TimerEvent &event);
  void tBikeCoreControl();

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_imu_ch100_msg_, sub_momentum_wheel_parsed_msg_;
  ros::Subscriber sub_remote_ctrl_msg_, sub_can_src_msg_;
  ros::Timer bike_core_control_timer_;
  std::shared_ptr<bike_core::remote_control_msg> rc_ctrl_msg_ptr_;
  std::shared_ptr<bike_core::odrive_can_msg> odrive_src_can_msg_ptr_;
  std::shared_ptr<bike_core::odrive_motor_feedback_msg>
      odrive_can_parsed_msg_ptr_;
};

#endif  // BIKEXUAN_CONTROL_HPP