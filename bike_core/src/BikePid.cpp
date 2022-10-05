#include "bike_core/BikePid.hpp"

BikePid::BikePid() {
  LOG(INFO) << "Bike PID";
  const std::string speed_pid_file =
      "/home/hll/code_space/bike_ws/src/bike_core/params/bike_speed_pid.yaml";
  const std::string angle_pid_file =
      "/home/hll/code_space/bike_ws/src/bike_core/params/bike_angle_pid.yaml";
  const std::string angle_vel_pid_file =
      "/home/hll/code_space/bike_ws/src/bike_core/params/"
      "bike_angle_velocity_pid.yaml";
  speed_pid_ptr_ = std::make_shared<PidParams>(speed_pid_file);
  angle_pid_ptr_ = std::make_shared<PidParams>(angle_pid_file);
  angle_vel_pid_ptr_ = std::make_shared<PidParams>(angle_vel_pid_file);
}

float BikePid::operator()(const float target, const float current,
                          const PidParams::PidType PID_TYPE,
                          std::shared_ptr<PidParams> pid) const {
  switch (PID_TYPE) {
    case PidParams::PidType::INCREMENTAL: {
      // to be add
      break;
    }
    case PidParams::PidType::POSITION: {
      return CalculatePositionSpeedPid(target, current, pid);
    }
    default:
      break;
  }
}

const float BikePid::CalculatePositionSpeedPid(
    float target, float current, std::shared_ptr<PidParams> pid) const {
  pid->current_ = current;
  pid->target_ = target;
  pid->error_ = pid->target_ - pid->current_;
  pid->error_integral_ += pid->error_;

  double p_out = pid->kp_ * pid->error_;
  double i_out = pid->ki_ * pid->error_integral_;
  double d_out = pid->kd_ * (pid->error_ - pid->last_error_);

  pid->output_ = p_out + i_out + d_out;
  pid->last_error_ = pid->error_;

  LOG_IF(WARNING, 0) << std::setprecision(4)
                     << std::setiosflags(std::ios::fixed)
                     << setiosflags(std::ios::showpos) << "Pid.Kp: " << pid->kp_
                     << "\tPid.Ki: " << pid->ki_ << "\tPid.Kd: " << pid->kd_
                     << "\tPid.error: " << pid->error_
                     << "\tPid.Kd: " << pid->kd_
                     << "\tPid.Output: " << pid->output_;

  return pid->output_;
}
