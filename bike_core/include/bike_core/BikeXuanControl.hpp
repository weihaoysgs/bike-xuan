#ifndef BIKEXUAN_CONTROL_HPP
#define BIKEXUAN_CONTROL_HPP

#include "bike_core/CanReceiveSend.hpp"
#include "bike_core/odrive_can_msg.h"
#include "bike_core/odrive_motor_feedback_msg.h"
#include "bike_core/remote_control_msg.h"
#include "bike_core/BikePid.hpp"
#include "bike_core/pid_params_msg.h"
#include "bike_core/OdriveMotorConfig.hpp"
#include "bike_core/sbus_channels_msg.h"
#include "bike_vision/road_obstacle_msg.h"

struct ImuPose {
  ImuPose() : yaw_(0.0), roll_(0.0), pitch_(0.0){};
  ros::Time time_stamp_;
  tf2Scalar yaw_;
  tf2Scalar pitch_;
  tf2Scalar roll_;
};

class BikeXuanControl {
 public:
  // Constructor function
  BikeXuanControl();
  
  // Deconstructor function
  ~BikeXuanControl() { if (socket_can_fd_) close(socket_can_fd_); };

  // Check the subscriber message timestamp
  const bool ChechSubscriberMessageTimestamp() const;

  int FindNearestObstacleIndex(const bike_vision::road_obstacle_msg &msg) const;
  int FindNearestObstacleDistance(const bike_vision::road_obstacle_msg &msg) const;
  
  // Calculate angle vel speed pid
  void AngleVelocityPidControl();

  // Calculate angle pid
  void AnglePidControl();

  // Calculate speed pid
  void SpeedPidControl();
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
    if (msg->can_id == OdriveMotorConfig::getSigleInstance().axis0_send_receive_vel_position_can_id_)
      *odrive_axis0_can_parsed_msg_ptr_.get() = *msg;
    else if (msg->can_id == OdriveMotorConfig::getSigleInstance().axis1_send_receive_vel_position_can_id_)
      *odrive_axis1_can_parsed_msg_ptr_.get() = *msg;
    LOG_IF(WARNING, 0) << "Speed: " << msg->speed << " "
                       << "Position: " << msg->position
                       << " Can ID: " << msg->can_id;
  };
  void SubIMUCH100MessageCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    if (first_imu_msg_rece_)
    {
      first_imu_msg_rece_ = false;
      gyro_x_speed_ = last_gyro_speed_ = msg->angular_velocity.x;
    }
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

  void SubRoadObstacleMessageCallback(const bike_vision::road_obstacle_msg::ConstPtr &msg) {
    *road_obstacle_msg_ptr_ = *msg;
    LOG_IF(INFO, 0) << "num_objects: " << static_cast<uint16_t>(road_obstacle_msg_ptr_->num_objects);
    for (int i=0; i < road_obstacle_msg_ptr_->num_objects; i++) {
        LOG_IF(WARNING, 0) << "center_x: " << road_obstacle_msg_ptr_->center_x[i] << " "
                           << "center_y: " << road_obstacle_msg_ptr_->center_y[i] << " "
                           << "distance: " << road_obstacle_msg_ptr_->distance[i];
    }
  }

 private:
  // thread and ros timer to control the bike blance
  void tBikeCoreControl();
  void tServoControl();
  void tBalance();
  void tUpdate();
  void timerBalance(const ros::TimerEvent &event);
  void timerDriverWheelControll(const ros::TimerEvent &event);

private:
  float current_speed_;
  float gyro_x_speed_;
  float roll_angle_;
  int t_ms_{0}, t_2ms_{0}, t_10ms_{0}, t_100ms_{0};
  int socket_can_fd_; 
  bool first_imu_msg_rece_{true};
  float last_gyro_speed_{0.0};
  bool cal_angle_vel_pid_ = false, cal_angle_pid_ = false, cal_speed_pid_ = false;
  double bike_roll_balance_angle_{0.0};
  float faucet_direction_{0.0}; 
  double avoid_obstacle_drive_speed_{0.0};
private:
  float angle_vel_pid_out_{0.0};
  float angle_pid_out_{0.0};
  float speed_pid_out_{0.0};
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_imu_ch100_msg_, sub_momentum_wheel_parsed_msg_;
  ros::Subscriber sub_remote_ctrl_msg_, sub_can_src_msg_;
  ros::Subscriber sub_road_obstacle_msg_;
  ros::Publisher pub_sbus_channels_value_;
  ros::Timer bike_core_control_timer_;
  ros::Timer bike_balance_timer_;
  ros::Timer bike_driver_wheel_control_timer_;
  std::shared_ptr<bike_core::remote_control_msg> rc_ctrl_msg_ptr_;
  std::shared_ptr<bike_core::odrive_can_msg> odrive_src_can_msg_ptr_;
  std::shared_ptr<sensor_msgs::Imu> bike_xuan_imu_msg_ptr_;
  std::shared_ptr<ImuPose> imu_ch100_pose_ptr_;
  std::shared_ptr<BikePid> bike_pid_ptr_;
  std::shared_ptr<bike_core::odrive_motor_feedback_msg>
      odrive_axis0_can_parsed_msg_ptr_, odrive_axis1_can_parsed_msg_ptr_;
  std::shared_ptr<bike_vision::road_obstacle_msg> road_obstacle_msg_ptr_;
};

#endif  // BIKEXUAN_CONTROL_HPP