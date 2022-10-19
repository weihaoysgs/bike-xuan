#include "bike_core/BikeXuanControl.hpp"
float target_speed_;
float current_speed;
float gyro_x_speed;
float roll_angle;

BikeXuanControl::BikeXuanControl() : nh_("~") {
  const std::string can_port_name = "can0";
  socket_can_fd_ =
      CanSendReceive::GetOneSocketCanSendInstance(can_port_name.c_str());
  LOG_IF(FATAL, !socket_can_fd_) << "Get Socket Can Instance Erros!";

  rc_ctrl_msg_ptr_ = std::make_shared<bike_core::remote_control_msg>();
  odrive_src_can_msg_ptr_ = std::make_shared<bike_core::odrive_can_msg>();
  bike_xuan_imu_msg_ptr_ = std::make_shared<sensor_msgs::Imu>();
  imu_ch100_pose_ptr_ = std::make_shared<ImuPose>();
  bike_pid_ptr_ = std::make_shared<BikePid>();
  odrive_can_parsed_msg_ptr_ =
      std::make_shared<bike_core::odrive_motor_feedback_msg>();

  sub_can_src_msg_ = nh_.subscribe<bike_core::odrive_can_msg>(
      "topic1", 100, &BikeXuanControl::SubOdriveCanSourceMessageCB, this);
  sub_remote_ctrl_msg_ = nh_.subscribe<bike_core::remote_control_msg>(
      "/parser_remote_data_node/remote_ctrl_data", 10,
      &BikeXuanControl::SubRemoteControlMessageCB, this);
  sub_momentum_wheel_parsed_msg_ =
      nh_.subscribe<bike_core::odrive_motor_feedback_msg>(
          "/can_send_receive_node/odrive_motor_parsed_data", 10,
          &BikeXuanControl::SubOdriveMotorFeedbackParsedMessageCB, this);

  sub_imu_ch100_msg_ = nh_.subscribe<sensor_msgs::Imu>(
      "/IMU_data", 1, &BikeXuanControl::SubIMUCH100MessageCallback, this);

  bike_balance_timer_ = nh_.createTimer(ros::Duration(1.0 / 1000),
                                        &BikeXuanControl::timerBalance, this);

  std::thread t_bike_core_control =
      std::thread(&BikeXuanControl::tBikeCoreControl, this);
  t_bike_core_control.detach();

  std::thread t_bike_balance = std::thread(&BikeXuanControl::tBalance, this);
  t_bike_balance.detach();

  std::thread update = std::thread(&BikeXuanControl::tUpdate, this);
  update.detach();
}

void BikeXuanControl::tUpdate() {
  while (ros::ok()) {
    gyro_x_speed_ = (bike_xuan_imu_msg_ptr_->angular_velocity.x * 0.7 +
                     last_gyro_speed_ * 0.3) *
                    100.0;
    last_gyro_speed_ = bike_xuan_imu_msg_ptr_->angular_velocity.x;
    roll_angle_ = radian2angle(imu_ch100_pose_ptr_->roll_);
    current_speed_ = odrive_can_parsed_msg_ptr_->speed * 10.0;
  }
}

/**
 * \brief Calculate angle vel pid per 2ms
 */
void BikeXuanControl::AngleVelocityPidControl() {
  // 调试角速度环的时候，正常的一个 *100 倍之后的输入是 [-4,4] 左右，为 IMU
  // 原始输入
  angle_vel_pid_out_ =
      (*bike_pid_ptr_)(angle_pid_out_, gyro_x_speed_, PidParams::POSITION,
                       bike_pid_ptr_->getAngleVelocityPid(),
                       bike_pid_ptr_->getAngleVelocityPid()->debug_);

  int16_t int_speed = static_cast<int16_t>(angle_vel_pid_out_);
  if (rc_ctrl_msg_ptr_->s1 == 3) {
    CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd_, 524,
                                                           int_speed);
  } else {
    CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd_, 524,
                                                           0);
  }
}

/**
 * \brief Calculate angle pid per 10ms
 */
void BikeXuanControl::AnglePidControl() {
  // the param should get in the system init
  constexpr float roll_balance_angle_ = 3.6;
  // 角度环的输出是速度环的输入，角度环的输出目前大概在 [-8,8] 左右
  // 输入大概在 [-3,3] 左右，调试时的输入为 IMU 原始输入
  angle_pid_out_ = (*bike_pid_ptr_)(
      roll_balance_angle_, roll_angle_ - speed_pid_out_, PidParams::POSITION,
      bike_pid_ptr_->getAnglePid(), bike_pid_ptr_->getAnglePid()->debug_);
}

/**
 * \brief Calculate speed pid per 10ms
 */
void BikeXuanControl::SpeedPidControl() {
  // 当前反馈的速度 *10 大概在 [-300,300] 之间
  speed_pid_out_ = (*bike_pid_ptr_)(0.0, current_speed_, PidParams::POSITION,
                                    bike_pid_ptr_->getSpeedPid(),
                                    bike_pid_ptr_->getSpeedPid()->debug_);
}

void BikeXuanControl::tBalance() {
  ros::Rate rate(10);  // hz
  while (ros::ok()) {
    if (cal_angle_vel_pid_ == 1) {
      AngleVelocityPidControl();
      cal_angle_vel_pid_ = 0;
    }
    if (cal_angle_pid_ == 1) {
      AnglePidControl();
      cal_angle_pid_ = 0;
    }
    if (cal_speed_pid_ == 1) {
      SpeedPidControl();
      cal_speed_pid_ = 0;
    }
  }
}

void BikeXuanControl::timerBalance(const ros::TimerEvent &event) {
  // find which pid's calculate time is max, let the t_ms_ = 0;
  std::map<int, std::string, std::greater<int>> map_pid_{
      {bike_pid_ptr_->getSpeedPid()->calculate_time_,
       bike_pid_ptr_->getSpeedPid()->pid_name_},
      {bike_pid_ptr_->getAnglePid()->calculate_time_,
       bike_pid_ptr_->getAnglePid()->pid_name_},
      {bike_pid_ptr_->getAngleVelocityPid()->calculate_time_,
       bike_pid_ptr_->getAngleVelocityPid()->pid_name_},
  };
  std::string max_cal_time_pid_ = map_pid_.begin()->second;
  t_ms_++;
  if (t_ms_ % bike_pid_ptr_->getSpeedPid()->calculate_time_ == 0) {
    cal_speed_pid_ = 1;
    if (max_cal_time_pid_ == bike_pid_ptr_->getSpeedPid()->pid_name_) t_ms_ = 0;
  }
  if (t_ms_ % bike_pid_ptr_->getAnglePid()->calculate_time_ == 0) {
    cal_angle_pid_ = 1;
    if (max_cal_time_pid_ == bike_pid_ptr_->getAnglePid()->pid_name_) t_ms_ = 0;
  }
  if (t_ms_ % bike_pid_ptr_->getAngleVelocityPid()->calculate_time_ == 0) {
    cal_angle_vel_pid_ = 1;
    if (max_cal_time_pid_ == bike_pid_ptr_->getAngleVelocityPid()->pid_name_)
      t_ms_ = 0;
  }
}

void BikeXuanControl::tBikeCoreControl() {
  /*
  const std::string can_port_name = "can0";
  const int socket_can_fd =
      CanSendReceive::GetOneSocketCanSendInstance(can_port_name.c_str());

  LOG_IF(FATAL, !socket_can_fd) << "Get Socket Can Instance Erros!";

  // TODO using yaml to save params
  const double control_rate = 100;
  const double tolerance_msg_dt = 0.1;
  geometry_msgs::Vector3 &gyro_msg = bike_xuan_imu_msg_ptr_->angular_velocity;
  ros::Rate rate(control_rate);

  constexpr double balance_roll_angle = 6.8;

  BikePid bike_pid;
  unsigned int count = 0;
  while (ros::ok()) {
    count++;
    if (count > 60000) count = 0;
    // TODO set the target motor speed 0.0
    LOG_IF(FATAL, !ChechSubscriberMessageTimestamp())
        << "ChechSubscriberMessageTimestamp Failed!\t" <<
        []() -> std::string { return std::string("Set Motor Speed To [0.0]"); };

    float target_remote_speed = rc_ctrl_msg_ptr_->ch_x[0] / 2.2;


    const float balance_roll_anle = 2.01;
    ////////////////////////////////////////////////////////////////////
    // 测速范围在 +-300

    speed_pid_out_ = bike_pid(1.0, current_speed_, PidParams::POSITION,
                              bike_pid.getSpeedPid(), 1);

    // if (std::abs(roll_angle_ - balance_roll_anle) > 3.0) roll_angle_ = 2.17;

    angle_pid_out_ = bike_pid(balance_roll_anle, roll_angle_ - speed_pid_out_,
                              PidParams::POSITION, bike_pid.getAnglePid(), 0);
    // 角速度环的输入限制在 [-10,10] 上
    angle_vel_pid_out_ =
        bike_pid(angle_pid_out_, gyro_x_speed_, PidParams::POSITION,
                 bike_pid.getAngleVelocityPid(), 0);

    ////////////////////////////////////////////////////////////////////
    int16_t int_speed = static_cast<int16_t>(angle_vel_pid_out_);
    if (rc_ctrl_msg_ptr_->s1 == 3)
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                             int_speed);
    else
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                             0);


    rate.sleep();
  }
  */
}

const bool BikeXuanControl::ChechSubscriberMessageTimestamp() const {
  double time_now = ros::Time::now().toSec();
  double remote_ctrl_data_time = rc_ctrl_msg_ptr_->header.stamp.toSec();
  double motor_parsed_data_time =
      odrive_can_parsed_msg_ptr_->header.stamp.toSec();
  LOG_IF(WARNING, 0) << "dt remote_ctrl_data_time: "
                     << std::abs(time_now - remote_ctrl_data_time)
                     << "  dt motor_parsed_data_time: "
                     << std::abs(time_now - motor_parsed_data_time);
  return true;
}