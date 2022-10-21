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
    odrv_config_file.release();

    axis0_send_receive_vel_position_can_id_ = get_odrv0_can_send_id(
        GET_ENCODER_ESTIMATES, axis0_motor_config_can_id_);
    axis1_send_receive_vel_position_can_id_ = get_odrv0_can_send_id(
        GET_ENCODER_ESTIMATES, axis1_motor_config_can_id_);
    axis0_set_input_pos_can_id_ =
        get_odrv0_can_send_id(SET_IUPUT_POS, axis0_motor_config_can_id_);
    axis1_set_input_pos_can_id_ =
        get_odrv0_can_send_id(SET_IUPUT_POS, axis1_motor_config_can_id_);
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
};

#endif  // ODRIVE_MOTOR_CONFIG_HPP
