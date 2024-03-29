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

  avoid_obstacle_drive_speed_ =
      OdriveMotorConfig::getSigleInstance().avoid_obstacle_drive_speed_;
  LOG(INFO) << "Avoid.Obstacle.Drive.Speed: " << avoid_obstacle_drive_speed_;

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

int BikeXuanControl::FindNearestObstacleIndex(const bike_vision::road_obstacle_msg &msg) const
{
  float temp_value = msg.distance[0];
  int temp_index = 0;
  for (int i=1; i<msg.distance.size(); i++) {
    if (msg.distance[i] < temp_value && msg.distance[i] != 0.0) {
      temp_value = msg.distance[i];
      temp_index = i;
    }
  }
  return temp_index;
}

void BikeXuanControl::tServoControl() {
  auto output_limit = [](float value, float min, float max) -> float {
    if (value > max)
      return max;
    else if (value < min)
      return min;
    else
      return value;
  };
  const double go_stright_dis = 4.0;
  const float tolerance_nearest_dis =
      OdriveMotorConfig::getSigleInstance().tolerance_nearest_obstacle_dis_;
  const float dir_error_p =
      OdriveMotorConfig::getSigleInstance().faucet_dir_error_p_;
  const float frame_center_x = 320.0, frame_center_y = 240.0;
  float near_obj_center_x, near_obj_center_y, near_obj_distacne, last_nearest_obstacle_dis;
  const uint16_t dir_control_rate =
      OdriveMotorConfig::getSigleInstance().faucet_dir_control_rate_;
  float no_obstacle_time_pwm_value;
  double distance_changed_value = 0.0;

  bike_core::sbus_channels_msg sbus_output_data;
  uint16_t faucet_dir_output =
      OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_;
  ros::Rate rate(dir_control_rate);
  bool flag_turn_left1_or_right0{true};
  float turn_right_pwm_value = 1560, turn_left_pwm_value = 1840;
  bool start_timer{false};
  float change_dir_time_period = 3.0; // s
  uint32_t period_changle_dir_times = static_cast<uint32_t>(change_dir_time_period / (1.0 / dir_control_rate));
  LOG(INFO) << "period_changle_dir_times: " << period_changle_dir_times;
  
  while (ros::ok()) {
    
    if (rc_ctrl_msg_ptr_->s2 == 3) {
      
      if (road_obstacle_msg_ptr_->num_objects) {
        // find the nearest obstacle
        int near_obj_index = FindNearestObstacleIndex(*road_obstacle_msg_ptr_);
        near_obj_center_x = road_obstacle_msg_ptr_->center_x[near_obj_index];
        near_obj_center_y = road_obstacle_msg_ptr_->center_y[near_obj_index];
        near_obj_distacne = road_obstacle_msg_ptr_->distance[near_obj_index];
        if (near_obj_distacne == 0) 
        {
          near_obj_distacne = last_nearest_obstacle_dis;
          LOG(INFO) << "near_obj_distacne==0";
        }
        distance_changed_value = last_nearest_obstacle_dis - near_obj_distacne;
        last_nearest_obstacle_dis = near_obj_distacne;
        if (distance_changed_value < -1.2)
        {
          LOG(INFO) << "distance_changed_value : " << distance_changed_value;
          flag_turn_left1_or_right0 = !flag_turn_left1_or_right0;
          LOG_IF(INFO, flag_turn_left1_or_right0) << "Next Turn Left";
          LOG_IF(INFO, !flag_turn_left1_or_right0) << "Next Turn Right";
        }

        //-------------------------------
        // if (rc_ctrl_msg_ptr_->ch_x[2] > 0)
        // {
        //   flag_turn_left1_or_right0 = 0;
        //   LOG(INFO) << "Next Turn Right";
        // }
        // if (rc_ctrl_msg_ptr_->ch_x[2] < 0)
        // {
        //   flag_turn_left1_or_right0 = 1;
        //   LOG(INFO) << "Next Turn Left";
        // }
        //-------------------------------

        LOG_IF(INFO, 0) << "distance_changed_value : " << distance_changed_value;
        if (near_obj_distacne < tolerance_nearest_dis) {
          
          // avoid the front obstacle
          float target_faucet_dir;
          if (flag_turn_left1_or_right0 == true)
          {
            target_faucet_dir = turn_left_pwm_value;
          }
          else{
            target_faucet_dir = turn_right_pwm_value;
          }
          
          float tar_cur_faucet_error = target_faucet_dir - faucet_direction_;
          if (tar_cur_faucet_error > 4) {
            faucet_dir_output += 3.0;
          } else if (tar_cur_faucet_error < -4) {
            faucet_dir_output -= 3.0;
          } else {
            faucet_dir_output = target_faucet_dir;
          }
          faucet_dir_output = output_limit(faucet_dir_output, 1500, 1900);
          LOG_IF(INFO, 1) << "Distance Less Sepcial Value, Turning:" << faucet_dir_output;
        } else if ( near_obj_distacne < go_stright_dis && 
                    near_obj_distacne > tolerance_nearest_dis) {
          // calculate x axis error, bike turn left, sould add, turn right, should desc
          float dir_error = frame_center_x - near_obj_center_x;

          float target_faucet_dir =
              OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_ +
              dir_error * dir_error_p;
          float tar_cur_faucet_error = target_faucet_dir - faucet_direction_;
          if (tar_cur_faucet_error > 4) {
            faucet_dir_output += 3.0;
          } else if (tar_cur_faucet_error < -4) {
            faucet_dir_output -= 3.0;
          } else {
            faucet_dir_output = target_faucet_dir;
          }
          faucet_dir_output = output_limit(faucet_dir_output, 1500, 1900);
          LOG_IF(INFO, 1) << "Keep One Line With Obstacle:" << faucet_dir_output;
        }
        
        {
          avoid_obstacle_drive_speed_ =
              OdriveMotorConfig::getSigleInstance().avoid_obstacle_drive_speed_;
          no_obstacle_time_pwm_value = faucet_dir_output;
        }

      }
      // now, no obstacle, turn dir to the last dir on the contrary
      else {
        float middle_angle =
            OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_;
        float target_faucet_dir = middle_angle;
        if (no_obstacle_time_pwm_value < middle_angle) {
          target_faucet_dir = middle_angle + 100;
        } else if (no_obstacle_time_pwm_value > middle_angle) {
          target_faucet_dir = middle_angle - 100;
        }

        float tar_cur_faucet_error = target_faucet_dir - faucet_direction_;
        if (tar_cur_faucet_error > 4) {
          faucet_dir_output += 3.0;
        } else if (tar_cur_faucet_error < -4) {
          faucet_dir_output -= 3.0;
        } else {
          faucet_dir_output = target_faucet_dir;
        }
        faucet_dir_output = output_limit(faucet_dir_output, 1500, 1900);
        LOG_IF(WARNING, 1) << "No Obstacle on the contrary with last turn dir";
      }

      {
        faucet_direction_ = faucet_dir_output;
        // control the faucet dir
        LOG_IF(WARNING, 0) << "faucet dir: " << faucet_direction_;
        sbus_output_data.channels_value[0] = faucet_direction_;
        if (OdriveMotorConfig::getSigleInstance().debug_faucet_dir_)
          pub_sbus_channels_value_.publish(sbus_output_data);
      }
    }
    rate.sleep();
  }
}

/**
 * \brief ros timer control the back wheel drive
 */
void BikeXuanControl::timerDriverWheelControll(const ros::TimerEvent &event) {
  // auto avoid obstacle control
  if ((rc_ctrl_msg_ptr_->s2 == 3 && rc_ctrl_msg_ptr_->s1 != 3) ||
      (rc_ctrl_msg_ptr_->s2 == 3 && rc_ctrl_msg_ptr_->s1 == 3)) {
    float drive_wheel_current = odrive_axis1_can_parsed_msg_ptr_->speed;

    float drive_speed_pid_out = (*bike_pid_ptr_)(
        avoid_obstacle_drive_speed_, drive_wheel_current, PidParams::POSITION,
        bike_pid_ptr_->getDriveWheelSpeedPid(),
        bike_pid_ptr_->getDriveWheelSpeedPid()->debug_);

    int16_t int_speed = static_cast<int16_t>(drive_speed_pid_out);

    CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
        socket_can_fd_,
        OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_,
        int_speed);
  }

  // normal debug control
  else if (rc_ctrl_msg_ptr_->s1 == 3 && rc_ctrl_msg_ptr_->s2 != 3) {
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

  else {
    CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
        socket_can_fd_,
        OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_, 0);
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

  while (!OdriveMotorConfig::getSigleInstance().config_init_success_)
    ;

  const double init_bike_middle_angle =
      OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_;
  const double limit_middle_angle_change_value = 
      OdriveMotorConfig::getSigleInstance().middle_angle_recitfy_limit_;
  const double max_bike_xuan_balance_angle = init_bike_middle_angle + limit_middle_angle_change_value;
  const double min_bike_xuan_balance_angle = init_bike_middle_angle - limit_middle_angle_change_value;
  LOG(INFO) << "max_bike_xuan_balance_angle: " << max_bike_xuan_balance_angle;
  LOG(INFO) << "min_bike_xuan_balance_angle: " << min_bike_xuan_balance_angle;
  
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
        !(count %
          OdriveMotorConfig::getSigleInstance().middle_angle_rectify_time_)) {
      float speed_balance_roll_angle_p =
          OdriveMotorConfig::getSigleInstance().angle_rectify_scale_;
      float dynamic_roll_change = current_speed_ * speed_balance_roll_angle_p;

      OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ +=
          dynamic_roll_change;
    
      if (OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ > max_bike_xuan_balance_angle){
          OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ = max_bike_xuan_balance_angle;
          LOG(WARNING) << "Bike Middle Angle Larger \"max_bike_xuan_balance_angle\"";
      }
      else if (OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ < min_bike_xuan_balance_angle){
          OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_ = min_bike_xuan_balance_angle;
          LOG(WARNING) << "Bike Middle Angle Less \"min_bike_xuan_balance_angle\"";
      }
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