#include "bike_core/OdriveMotorConfig.hpp"
void InitGlog() {
  google::InitGoogleLogging("TestBikePid");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odrv0_config_test_node");
  std::cout << OdriveMotorConfig::getSigleInstance()
                   .axis0_send_receive_vel_position_can_id_
            << std::endl;
  std::cout << OdriveMotorConfig::getSigleInstance()
                   .axis1_send_receive_vel_position_can_id_
            << std::endl;
  std::cout << OdriveMotorConfig::getSigleInstance().axis0_set_input_pos_can_id_
            << std::endl;
  std::cout << OdriveMotorConfig::getSigleInstance().axis1_set_input_pos_can_id_
            << std::endl;
  if (OdriveMotorConfig::getSigleInstance().debug_run_back_drive_wheel_) {
    std::cout << "debug_run_back_drive_wheel_" << std::endl;
  }
  if (OdriveMotorConfig::getSigleInstance().debug_run_momentum_wheel_) {
    std::cout << "debug_run_momentum_wheel_" << std::endl;
  }
  std::cout << "IMU.Machine.Middle.Angle:"
            << OdriveMotorConfig::getSigleInstance().imu_machine_middle_angle_
            << std::endl;
  std::cout << "Servo.Port.Name: "
            << OdriveMotorConfig::getSigleInstance().servo_port_name_
            << std::endl;
  std::cout << "Servo.ID:" << OdriveMotorConfig::getSigleInstance().servo_id_
            << std::endl;
  std::cout << "Servo.BaudRate: "
            << OdriveMotorConfig::getSigleInstance().servo_port_baud_rate_
            << std::endl;
  std::cout << "Servo.PWM.Middle.Value: "
            << OdriveMotorConfig::getSigleInstance().servo_pwm_middle_angle_
            << std::endl;
  std::cout << "Output.Middle.Angle: "
            << OdriveMotorConfig::getSigleInstance().output_imu_middle_angle_
            << std::endl;
  std::cout << "Bike.Tunr.Scale: "
            << OdriveMotorConfig::getSigleInstance().bike_turn_scale_
            << std::endl;
  std::cout << "Bike.Angle.Rectify.Scale: "
            << OdriveMotorConfig::getSigleInstance().angle_rectify_scale_
            << std::endl;
  std::cout << "Bike.Angle.Rectify.Calculate.Time: "
            << OdriveMotorConfig::getSigleInstance().middle_angle_rectify_time_
            << std::endl;
  std::cout << "Debug.Faucet.Dir "
            << OdriveMotorConfig::getSigleInstance().debug_faucet_dir_
            << std::endl;
  std::cout << "Avoid.Obstacle.Drive.SPeed: "
            << OdriveMotorConfig::getSigleInstance().avoid_obstacle_drive_speed_
            << std::endl;
  std::cout << "Tolerance.Nearest.Obstacle.Dis: "
            << OdriveMotorConfig::getSigleInstance().tolerance_nearest_obstacle_dis_
            << std::endl;
  std::cout << std::endl;
  ros::spin();
}