#include "bike_core/CanReceiveSend.hpp"
void InitGlog() {
  google::InitGoogleLogging("can_send_receive_node");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}
int main(int argc, char** argv) {
  InitGlog();
  ros::init(argc, argv, "can_send_receive_node");
  CanSendReceive can;
  ros::spin();
  return 0;
}