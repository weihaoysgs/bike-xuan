#include <glog/logging.h>
#include <ros/ros.h>

#include "bike_core/SbusSimulate.hpp"
#include "qapplication.h"

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
  QApplication app(argc, argv);
  InitGlog();
  ros::init(argc, argv, "subs_simulate_demo");
  SbusSimulateSerial sbus_simulate;
  ros::spin();
  return 0;
}