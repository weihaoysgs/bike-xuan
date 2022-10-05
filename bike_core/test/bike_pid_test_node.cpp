#include <ros/ros.h>

#include "bike_core/BikePid.hpp"
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
  InitGlog();
  ros::init(argc, argv, "bike_pid_test_node");
  // BikePid bike_pid;
  // float test = bike_pid(0, 0);
  ros::spin();
  return 0;
}