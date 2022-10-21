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
  std::cout << OdriveMotorConfig::getSigleInstance()
                   .axis0_set_input_pos_can_id_
            << std::endl;
  std::cout << OdriveMotorConfig::getSigleInstance()
                   .axis1_set_input_pos_can_id_
            << std::endl;
  ros::spin();
}