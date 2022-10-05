#include "bike_core/BikePid.hpp"

BikePid::BikePid() {
  LOG(INFO) << "Bike PID";
  const std::string speed_pid_file = "/home/hll/code_space/bike_ws/src/bike_core/params/bike_speed_pid.yaml";
  speed_pid_ptr_ = std::make_shared<PidParams>(speed_pid_file);
}

float BikePid::operator()(const float target, float current) const {
  LOG(INFO) << "calculate PID";
}
