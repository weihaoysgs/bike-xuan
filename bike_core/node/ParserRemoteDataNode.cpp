#include "bike_core/RemoteControlParser.hpp"

void InitGlog() {
  google::InitGoogleLogging("RemoteControlNode");
  google::SetLogFilenameExtension("log_");
  google::SetLogDestination(google::INFO, "log/");

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 1024;
  FLAGS_stop_logging_if_full_disk = true;
}
int main(int argc, char** argv) {
  InitGlog();
  QApplication a(argc, argv);
  ros::init(argc, argv, "parser_remote_data_node");
  RemoteControlDataParser parser;
  return QApplication::exec();
} 