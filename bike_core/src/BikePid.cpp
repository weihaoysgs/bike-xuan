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

float BikePid::operator()(const float target, float current) const {
  LOG(INFO) << "calculate PID";
}

const float BikePid::CalculateBikeXxuanSpeedPid(float target,
                                                float current) const {
  speed_pid_ptr_->current_ = current;
  speed_pid_ptr_->target_ = target;
  speed_pid_ptr_->error_ = speed_pid_ptr_->target_ - speed_pid_ptr_->current_;
  speed_pid_ptr_->error_integral_ += speed_pid_ptr_->error_;

  double p_out = speed_pid_ptr_->kp_ * speed_pid_ptr_->error_;
  double i_out = speed_pid_ptr_->ki_ * speed_pid_ptr_->error_integral_;
  double d_out = speed_pid_ptr_->kd_ *
                 (speed_pid_ptr_->error_ - speed_pid_ptr_->last_error_);

  speed_pid_ptr_->output_ = p_out + i_out + d_out;
  speed_pid_ptr_->last_error_ = speed_pid_ptr_->error_;

  LOG_IF(WARNING, 0) << std::setprecision(4) << std::setiosflags(std::ios::fixed)
                     << setiosflags(std::ios::showpos)
                     << "Pid.Kp: " << speed_pid_ptr_->kp_
                     << "\tPid.Ki: " << speed_pid_ptr_->ki_
                     << "\tPid.Kd: " << speed_pid_ptr_->kd_
                     << "\tPid.error: " << speed_pid_ptr_->error_
                     << "\tPid.Kd: " << speed_pid_ptr_->kd_
                     << "\tPid.Output: " << speed_pid_ptr_->output_;

  return speed_pid_ptr_->output_;
}
