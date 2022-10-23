#include "bike_core/ServoControl.hpp"

void InitGlog() {
  google::InitGoogleLogging("TestServoControl");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}

int main(int argc, char** argv) {
  InitGlog();
  ros::init(argc, argv, "test_servo_control_node");
  ServoControl::getSingleInstance().SetSerilaServoAngle(90.0);
  LOG(INFO) << "Hello Servo Control Node";
  ros::spin();
  return 0;
}