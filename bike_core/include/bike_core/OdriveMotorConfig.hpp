#ifndef ODRIVE_MOTOR_CONFIG_HPP
#define ODRIVE_MOTOR_CONFIG_HPP

#include <glog/logging.h>
#include <ros/ros.h>

#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

class OdriveMotorConfig {
 public:
  OdriveMotorConfig(const std::string file_path) {
    cv::FileStorage odrv_config_file(file_path, cv::FileStorage::READ);
    LOG_IF(FATAL, !odrv_config_file.isOpened())
        << file_path << "\t Is Open Error";
    odrv_config_file["Axis0.Can.ID"] >> axis0_motor_config_can_id_;
    odrv_config_file["Axis1.Can.ID"] >> axis1_motor_config_can_id_;
    odrv_config_file["Debug.Momentum.Wheel"] >> debug_run_momentum_wheel_;
    odrv_config_file["Debug.Back.Drive.Wheel"] >> debug_run_back_drive_wheel_;
    odrv_config_file["IMU.Machine.Middle.Angle"] >> imu_machine_middle_angle_;
    odrv_config_file["Servo.Port.Name"] >> servo_port_name_;
    odrv_config_file["Servo.ID"] >> servo_id_;
    odrv_config_file["Servo.BaudRate"] >> servo_port_baud_rate_;
    odrv_config_file["Dbus.Serial.Port.Name"] >> dbus_serial_port_name_;
    odrv_config_file["Sbus.Serial.Port.Name"] >> sbus_serial_port_name_;
    odrv_config_file["Imu.Serial.Port.Name"] >> imu_serial_port_name_;
    odrv_config_file["Output.Middle.Angle"] >> output_imu_middle_angle_; 
    odrv_config_file["Servo.PWM.Middle.Value"] >> servo_pwm_middle_angle_;
    odrv_config_file["Bike.Turn.Scale"] >> bike_turn_scale_;
    odrv_config_file["Bike.Middle.Angle.Rectify.Scale"] >> angle_rectify_scale_;
    odrv_config_file["Middle.Angle.Rectify.Calculate.Time"] >> middle_angle_rectify_time_;
    odrv_config_file["Debug.Faucte.Dir"] >> debug_faucet_dir_;
    odrv_config_file["Avoid.Obstacle.Drive.Speed"] >> avoid_obstacle_drive_speed_;
    odrv_config_file["Tolerance.Nearest.Obstacle.Dis"] >> tolerance_nearest_obstacle_dis_;
    odrv_config_file.release();

    axis0_send_receive_vel_position_can_id_ = get_odrv0_can_send_id(
        GET_ENCODER_ESTIMATES, axis0_motor_config_can_id_);
    axis1_send_receive_vel_position_can_id_ = get_odrv0_can_send_id(
        GET_ENCODER_ESTIMATES, axis1_motor_config_can_id_);
    axis0_set_input_pos_can_id_ =
        get_odrv0_can_send_id(SET_IUPUT_POS, axis0_motor_config_can_id_);
    axis1_set_input_pos_can_id_ =
        get_odrv0_can_send_id(SET_IUPUT_POS, axis1_motor_config_can_id_);

    config_init_success_ = true;
  }

  std::function<int(int, int)> get_odrv0_can_send_id =
      [](int cmd_id, int axis_id) -> int { return axis_id << 5 | cmd_id; };

 public:
  static OdriveMotorConfig& getSigleInstance() {
    static OdriveMotorConfig odrv0_config(
        "./src/bike_core/params/odrive_motor_config.yaml");
    return odrv0_config;
  }

 public:
  int axis0_motor_config_can_id_, axis1_motor_config_can_id_;
  int axis0_send_receive_vel_position_can_id_,
      axis1_send_receive_vel_position_can_id_;
  int axis0_set_input_pos_can_id_, axis1_set_input_pos_can_id_;
  const static int SET_IUPUT_POS = 0x00C;
  const static int REBOOT = 0x00C;
  const static int GET_ENCODER_ESTIMATES = 0x009;
  bool debug_run_momentum_wheel_{true}, debug_run_back_drive_wheel_{true};
  bool debug_faucet_dir_{false};
  bool output_imu_middle_angle_{false};
  int servo_pwm_middle_angle_{1700};
  double imu_machine_middle_angle_{0.0};
  double bike_turn_scale_{0.0};
  double avoid_obstacle_drive_speed_{0.0};
  double tolerance_nearest_obstacle_dis_{0.0};
  int servo_id_{-1};
  std::string servo_port_name_{""}, dbus_serial_port_name_{""},
      sbus_serial_port_name_{""}, imu_serial_port_name_{""};
  int servo_port_baud_rate_;
  double angle_rectify_scale_{0.0};
  bool config_init_success_{false};
  int middle_angle_rectify_time_{0};
};

#endif  // ODRIVE_MOTOR_CONFIG_HPP
