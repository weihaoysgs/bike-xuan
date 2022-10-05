#ifndef BIKE_PID_HPP
#define BIKE_PID_HPP

#include <glog/logging.h>

#include <memory>

struct PidParams {
  PidParams(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd){};
  float kp_{0.0}, ki_{0.0}, kd_{0.0};
  float last_error_{0.0};
};

class BikePid {
 public:
  explicit BikePid();
  ~BikePid() = default;
  float operator()(const float target, float current) const;

 public:
  static BikePid& getInstance() {
    static BikePid instance;
    return instance;
  }

 private:
  std::shared_ptr<PidParams> angle_pid_;
  std::shared_ptr<PidParams> angle_vel_pid_;
  std::shared_ptr<PidParams> speed_pid_;
};

#endif  // BIKE_PID_HPP