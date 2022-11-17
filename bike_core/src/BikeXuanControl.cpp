#include "bike_core/BikeXuanControl.hpp"

BikeXuanControl::BikeXuanControl() : nh_("~") {
  bike_roll_balance_angle_ =
      OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_;
  LOG(INFO) << "Bike.Machine.IMU.Middle.Angle: " << bike_roll_balance_angle_;
  const std::string can_port_name = "can0";
  socket_can_fd_ =
      CanSendReceive::GetOneSocketCanSendInstance(can_port_name.c_str());
  LOG_IF(FATAL, !socket_can_fd_) << "Get Socket Can Instance Erros!";

  // init member variable
  road_obstacle_msg_ptr_ = std::make_shared<bike_vision::road_obstacle_msg>();
  rc_ctrl_msg_ptr_ = std::make_shared<bike_core::remote_control_msg>();
  odrive_src_can_msg_ptr_ = std::make_shared<bike_core::odrive_can_msg>();
  bike_xuan_imu_msg_ptr_ = std::make_shared<sensor_msgs::Imu>();
  imu_ch100_pose_ptr_ = std::make_shared<ImuPose>();
  bike_pid_ptr_ = std::make_shared<BikePid>();
  odrive_axis0_can_parsed_msg_ptr_ =
      std::make_shared<bike_core::odrive_motor_feedback_msg>();
  odrive_axis1_can_parsed_msg_ptr_ =
      std::make_shared<bike_core::odrive_motor_feedback_msg>();

  // init publisher
  pub_sbus_channels_value_ =
      nh_.advertise<bike_core::sbus_channels_msg>("sbus_channel_values", 10);

  // init subscriber
  sub_road_obstacle_msg_ = nh_.subscribe<bike_vision::road_obstacle_msg>(
      "/bike_obstacle_info", 10,
      &BikeXuanControl::SubRoadObstacleMessageCallback, this);
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

  bike_driver_wheel_control_timer_ = nh_.createTimer(
      ros::Duration(bike_pid_ptr_->getDriveWheelSpeedPid()->calculate_time_ /
                    1000.0),
      &BikeXuanControl::timerDriverWheelControll, this);

  std::thread t_bike_core_control =
      std::thread(&BikeXuanControl::tBikeCoreControl, this);
  t_bike_core_control.detach();

  std::thread t_bike_balance = std::thread(&BikeXuanControl::tBalance, this);
  t_bike_balance.detach();

  std::thread update = std::thread(&BikeXuanControl::tUpdate, this);
  update.detach();

  std::thread t_servo_control =
      std::thread(&BikeXuanControl::tServoControl, this);
  t_servo_control.detach();
}

void BikeXuanControl::tServoControl() {}

/**
 * \brief ros timer control the back wheel drive
 */
void BikeXuanControl::timerDriverWheelControll(const ros::TimerEvent &event) {
  const float const_driver_speed = 2.0;
  const float frame_center_x = 320.0, frame_center_y = 240.0;
  faucet_direction_ =
      OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_;
  bike_core::sbus_channels_msg sbus_output_data;
  sbus_output_data.channels_value[0] = static_cast<uint16_t>(faucet_direction_);
  // pub_sbus_channels_value_.publish(sbus_output_data);
  float near_obj_center_x, near_obj_center_y, near_obj_distacne;

  // 此时切换自动避障模式
  if (rc_ctrl_msg_ptr_->s2 == 3) {
    // 如果此时检测到了障碍物
    if (road_obstacle_msg_ptr_->num_objects) {
      // 找到最近的那个障碍物的距离和位置信息
      // 1.0 此时只检测到一个障碍物
      if (road_obstacle_msg_ptr_->num_objects == 1) {
        near_obj_center_x = road_obstacle_msg_ptr_->center_x[0];
        near_obj_center_y = road_obstacle_msg_ptr_->center_y[0];
        near_obj_distacne = road_obstacle_msg_ptr_->distance[0];
      }
      // 此时同一个画面中检测到了多个障碍物
      else {
        int8_t near_obj_index = 0;
        near_obj_center_x = road_obstacle_msg_ptr_->center_x[near_obj_index];
        near_obj_center_y = road_obstacle_msg_ptr_->center_y[near_obj_index];
        near_obj_distacne = road_obstacle_msg_ptr_->distance[near_obj_index];
      }
      // 计算 x 轴的偏差
      float dir_error = frame_center_x - near_obj_center_x;

    }
    // 此时没检测到障碍物,直行
    else {
      LOG_IF(WARNING, 1) << "No Obstacle";
      pub_sbus_channels_value_.publish(sbus_output_data);
      float drive_wheel_target = const_driver_speed; 
      float drive_wheel_current = odrive_axis1_can_parsed_msg_ptr_->speed;
      float drive_speed_pid_out = (*bike_pid_ptr_)(
          drive_wheel_target, drive_wheel_current, PidParams::POSITION,
          bike_pid_ptr_->getDriveWheelSpeedPid(),
          bike_pid_ptr_->getDriveWheelSpeedPid()->debug_);

      int16_t int_speed = static_cast<int16_t>(drive_speed_pid_out);
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
          socket_can_fd_,
          OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_,
          int_speed);
    }
  }

/**
 * \brief ros timer control the back wheel drive
 */
void BikeXuanControl::timerDriverWheelControll(const ros::TimerEvent &event) {
  // normal debug control
  if (rc_ctrl_msg_ptr_->s1 == 3 && rc_ctrl_msg_ptr_->s2 != 3) {
    // control faucet turn angle
    faucet_direction_ =
        -rc_ctrl_msg_ptr_->ch_x[0] +
        OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_;
    bike_core::sbus_channels_msg sbus_output_data;
    sbus_output_data.channels_value[0] =
        static_cast<uint16_t>(faucet_direction_);
    pub_sbus_channels_value_.publish(sbus_output_data);

    float drive_wheel_target = rc_ctrl_msg_ptr_->ch_x[3] / 100.0;
    float drive_wheel_current = odrive_axis1_can_parsed_msg_ptr_->speed;
    // control back wheel
    float drive_speed_pid_out = (*bike_pid_ptr_)(
        drive_wheel_target, drive_wheel_current, PidParams::POSITION,
        bike_pid_ptr_->getDriveWheelSpeedPid(),
        bike_pid_ptr_->getDriveWheelSpeedPid()->debug_);

    int16_t int_speed = static_cast<int16_t>(drive_speed_pid_out);
    if (OdriveMotorConfig::getSigleInstance().debug_run_back_drive_wheel_) {
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
          socket_can_fd_,
          OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_,
          int_speed);
    } else {
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
          socket_can_fd_,
          OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_, 0);
    }
  }
}

void BikeXuanControl::tUpdate() {
  while (ros::ok()) {
    gyro_x_speed_ = (bike_xuan_imu_msg_ptr_->angular_velocity.x * 0.7 +
                     last_gyro_speed_ * 0.3) *
                    100.0;
    last_gyro_speed_ = bike_xuan_imu_msg_ptr_->angular_velocity.x;
    roll_angle_ = radian2angle(imu_ch100_pose_ptr_->roll_);
    current_speed_ = odrive_axis0_can_parsed_msg_ptr_->speed * 10.0;
  }
}

/**
 * \brief Calculate angle vel pid per 2ms
 */
void BikeXuanControl::AngleVelocityPidControl() {
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
  angle_pid_out_ =
      (*bike_pid_ptr_)(bike_roll_balance_angle_, roll_angle_ - speed_pid_out_,
                       PidParams::POSITION, bike_pid_ptr_->getAnglePid(),
                       bike_pid_ptr_->getAnglePid()->debug_);
}

/**
 * \brief Calculate speed pid per 10ms
 */
void BikeXuanControl::SpeedPidControl() {
  speed_pid_out_ = (*bike_pid_ptr_)(0.0, current_speed_, PidParams::POSITION,
                                    bike_pid_ptr_->getSpeedPid(),
                                    bike_pid_ptr_->getSpeedPid()->debug_);
}

void BikeXuanControl::tBalance() {
  ros::Rate rate(10);  // hz
  while (ros::ok()) {
    if (cal_angle_vel_pid_ == 1) {
      // AngleVelocityPidControl();
      cal_angle_vel_pid_ = 0;
    }
    if (cal_angle_pid_ == 1) {
      // AnglePidControl();
      cal_angle_pid_ = 0;
    }
    if (cal_speed_pid_ == 1) {
      // SpeedPidControl();
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
  const std::string can_port_name = "can0";
  const int socket_can_fd =
      CanSendReceive::GetOneSocketCanSendInstance(can_port_name.c_str());

  LOG_IF(FATAL, !socket_can_fd) << "Get Socket Can Instance Erros!";

  // TODO using yaml to save params
  const double control_rate = 1000;
  const double tolerance_msg_dt = 0.1;
  geometry_msgs::Vector3 &gyro_msg = bike_xuan_imu_msg_ptr_->angular_velocity;
  ros::Rate rate(control_rate);

  BikePid bike_pid;
  unsigned int count = 0;
  while (ros::ok()) {
    count++;
    if (count > 60000) count = 0;
    // TODO set the target motor speed 0.0
    LOG_IF(FATAL, !ChechSubscriberMessageTimestamp())
        << "ChechSubscriberMessageTimestamp Failed!\t" <<
        []() -> std::string { return std::string("Set Motor Speed To [0.0]"); };

    int pwm_middle_value =
        OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_;
    double turn_scale = OdriveMotorConfig::getSigleInstance().bike_turn_scale_;
    //////////////////////////////////////////////////////////////////
    float balance_roll_anle =
        OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_;
    balance_roll_anle += (faucet_direction_ - pwm_middle_value) * turn_scale;

    // dynamic config middle machine angle
    if (OdriveMotorConfig::getSigleInstance().middle_angle_rectify_time_ != 0 &&
        !(count % OdriveMotorConfig::getSigleInstance().middle_angle_rectify_time_)) {
      float speed_balance_roll_angle_p =
          OdriveMotorConfig::getSigleInstance().angle_rectify_scale_;
      float dynamic_roll_change = current_speed_ * speed_balance_roll_angle_p;

      OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ +=
          dynamic_roll_change;
    }

    if (count % bike_pid.getSpeedPid()->calculate_time_ == 0) {
      speed_pid_out_ =
          bike_pid(1.0, current_speed_, PidParams::POSITION,
                   bike_pid.getSpeedPid(), bike_pid.getSpeedPid()->debug_);
    }

    if (count % bike_pid.getAnglePid()->calculate_time_ == 0) {
      angle_pid_out_ = bike_pid(balance_roll_anle, roll_angle_ - speed_pid_out_,
                                PidParams::POSITION, bike_pid.getAnglePid(),
                                bike_pid.getAnglePid()->debug_);
    }

    if (count % bike_pid.getAngleVelocityPid()->calculate_time_ == 0) {
      angle_vel_pid_out_ =
          bike_pid(angle_pid_out_, gyro_x_speed_, PidParams::POSITION,
                   bike_pid.getAngleVelocityPid(),
                   bike_pid.getAngleVelocityPid()->debug_);
    }

    LOG_IF(INFO, OdriveMotorConfig::getSigleInstance().output_imu_middle_angle_)
        << "middle angle: " << roll_angle_;
    ////////////////////////////////////////////////////////////////////
    int16_t int_speed = static_cast<int16_t>(angle_vel_pid_out_);
    if (rc_ctrl_msg_ptr_->s1 == 3 &&
        OdriveMotorConfig::getSigleInstance().debug_run_momentum_wheel_)
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                             int_speed);
    else if (balance_roll_anle - roll_angle_ > std::abs(10))
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                             0);
    else
      CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                             0);

    rate.sleep();
  }
}

const bool BikeXuanControl::ChechSubscriberMessageTimestamp() const {
  double time_now = ros::Time::now().toSec();
  double remote_ctrl_data_time = rc_ctrl_msg_ptr_->header.stamp.toSec();
  double motor_parsed_data_time =
      odrive_axis0_can_parsed_msg_ptr_->header.stamp.toSec();
  LOG_IF(WARNING, 0) << "dt remote_ctrl_data_time: "
                     << std::abs(time_now - remote_ctrl_data_time)
                     << "  dt motor_parsed_data_time: "
                     << std::abs(time_now - motor_parsed_data_time);
  return true;
}