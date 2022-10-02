#include "bike_core/BikeXuanControl.hpp"
void InitGlog() {
  google::InitGoogleLogging("bike_core_control_node");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}
int main(int argc, char** argv) {
  InitGlog();
  ros::init(argc, argv, "bike_core_control_node");
  BikeXuanControl bike_xuan_core_control;
  ros::spin();
  return 0;
}