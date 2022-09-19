#include <glog/logging.h>
#include <ros/ros.h>

#include "bike_core/ImuCH100.hpp"

void InitGlog() {
  google::InitGoogleLogging("Remote Control Node");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}

int main(int argc, char** argv) {
  InitGlog();
  ros::init(argc, argv, "sub_imu_test_node");
  ImuCH100 imu;
  LOG(INFO) << "Hello Node";
  qDebug() << "Qt";
  ros::spin();
}