#ifndef BIKEXUAN_CONTROL_HPP
#define BIKEXUAN_CONTROL_HPP

#include "bike_core/Balance.hpp"
#include "bike_core/CanReceiveSend.hpp"
#include "bike_core/odrive_can_msg.h"
#include "bike_core/odrive_motor_feedback_msg.h"
#include "bike_core/remote_control_msg.h"
#include "bike_core/BikePid.hpp"
#include "bike_core/pid_params_msg.h"

struct ImuPose {
  ImuPose() : yaw_(0.0), roll_(0.0), pitch_(0.0){};
  ros::Time time_stamp_;
  tf2Scalar yaw_;
  tf2Scalar pitch_;
  tf2Scalar roll_;
};

class BikeXuanControl {
 public:
  BikeXuanControl();
  ~BikeXuanControl() = default;
  const bool ChechSubscriberMessageTimestamp() const;

 public:
  std::function<tf2Scalar(tf2Scalar)> radian2angle =
      [this](tf2Scalar radian) -> tf2Scalar { return radian * 180.0 / M_PI; };

 private:

  void SubPidParamsMessageCB(const bike_core::pid_params_msg::ConstPtr &msg);

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
                       << " Can ID: " << odrive_can_parsed_msg_ptr_->can_id;
  };
  void SubIMUCH100MessageCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    *bike_xuan_imu_msg_ptr_.get() = *msg;
    geometry_msgs::Quaternion_<std::allocator<void>> qtn_ =
        msg.get()->orientation;
    tf2::Matrix3x3 m(tf2::Quaternion(qtn_.x, qtn_.y, qtn_.z, qtn_.w));
    double roll, pitch, yaw;
    imu_ch100_pose_ptr_->time_stamp_ = msg->header.stamp;
    m.getRPY(imu_ch100_pose_ptr_->roll_, imu_ch100_pose_ptr_->pitch_,
             imu_ch100_pose_ptr_->yaw_);
    LOG_IF(WARNING, 0) << std::setprecision(3) << std::setiosflags(std::ios::fixed) 
                       << std::setiosflags(std::ios::left) << setiosflags(std::ios::showpos) 
        << std::setw(7) << "Acc x: " << std::setw(5) << bike_xuan_imu_msg_ptr_->linear_acceleration.x << " "
        << std::setw(7) << "Acc y: " << std::setw(5) << bike_xuan_imu_msg_ptr_->linear_acceleration.y << " "
        << std::setw(7) << "Acc z: " << std::setw(5) << bike_xuan_imu_msg_ptr_->linear_acceleration.z << " "
        << std::setw(7) << "Gyro x: " << std::setw(5) << bike_xuan_imu_msg_ptr_->angular_velocity.x  << " "
        << std::setw(7) << "Gyro y: " << std::setw(5) << bike_xuan_imu_msg_ptr_->angular_velocity.y  << " "
        << std::setw(7) << "Gyro z: " << std::setw(5) << bike_xuan_imu_msg_ptr_->angular_velocity.z  << " "
        << std::setw(7) << "Yaw: :" << std::setw(5) << radian2angle(imu_ch100_pose_ptr_->yaw_) << " "
        << std::setw(7) << "Pitch: :" << std::setw(5) << radian2angle(imu_ch100_pose_ptr_->pitch_) << " "
        << std::setw(7) << "Roll: :" << std::setw(5) << radian2angle(imu_ch100_pose_ptr_->roll_) << std::endl;
  };

 private:
  // void timerthreadBikeCoreControl(const ros::TimerEvent &event);
  void tBikeCoreControl();
  void tBalance();
  void tUpdate();
  void timerBalance(const ros::TimerEvent &event);

private:
  float current_speed_;
  float gyro_x_speed_;
  float roll_angle_;
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_imu_ch100_msg_, sub_momentum_wheel_parsed_msg_;
  ros::Subscriber sub_remote_ctrl_msg_, sub_can_src_msg_;
  ros::Timer bike_core_control_timer_;
  ros::Timer bike_balance_timer_;
  std::shared_ptr<bike_core::remote_control_msg> rc_ctrl_msg_ptr_;
  std::shared_ptr<bike_core::odrive_can_msg> odrive_src_can_msg_ptr_;
  std::shared_ptr<sensor_msgs::Imu> bike_xuan_imu_msg_ptr_;
  std::shared_ptr<ImuPose> imu_ch100_pose_ptr_;
  std::shared_ptr<bike_core::odrive_motor_feedback_msg>
      odrive_can_parsed_msg_ptr_;
};

#endif  // BIKEXUAN_CONTROL_HPP