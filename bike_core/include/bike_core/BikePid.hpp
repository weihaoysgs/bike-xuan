#ifndef BIKE_PID_HPP
#define BIKE_PID_HPP

#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <memory>
#include <opencv2/opencv.hpp>

struct PidParams {
  PidParams(float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd), pid_name_(""){};
  PidParams(const std::string& pid_parmas_file) {
    cv::FileStorage file(pid_parmas_file, cv::FileStorage::READ);
    LOG_IF(FATAL, !file.isOpened())
        << "File: " << pid_parmas_file << "Is Not Found";
    file["Kp"] >> kp_;
    file["Ki"] >> ki_;
    file["Kd"] >> kd_;
    file["Pid.Name"] >> pid_name_;
    file["Output.Limit"] >> output_limit_;
    file["Integal.Limit"] >> integral_limit_;
    file["Use.Integal.Limit"] >> use_intgral_limit_;
    file["Use.Output.Limit"] >> use_output_limit_;
    file["CalculateTime"] >> calculate_time_;
    file["Debug"] >> debug_;
    LOG(INFO) << pid_name_ << " param read complete!";
    LOG_IF(WARNING, 1) << "Kp: " << kp_ << "\tKi: " << ki_ << "\tKd: " << kd_
                       << "\tPid.Name: " << pid_name_
                       << "\tOutput.Limit: " << output_limit_;
  };
  enum PidType { INCREMENTAL, POSITION };
  ~PidParams() = default;
  std::string pid_name_;
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double last_error_{0.0}, error_{0.0}, error_integral_{0.0};
  double target_{0.0}, current_{0.0};
  double output_{0.0};
  double pid_error_integral_limit_{0.0};
  double output_limit_{0.0}, integral_limit_{0.0};
  bool use_intgral_limit_{false};
  bool use_output_limit_{true};
  int calculate_time_{10};
  bool debug_{false};
};

class BikePid {
 public:
  explicit BikePid();
  ~BikePid() = default;
  float operator()(const float target, const float current,
                   const PidParams::PidType PID_TYPE,
                   std::shared_ptr<PidParams> pid, bool debug) const;
  const float CalculatePositionSpeedPid(float target, float current,
                                        std::shared_ptr<PidParams> pid,
                                        bool debug) const;
  std::shared_ptr<PidParams> getAngleVelocityPid() const { return angle_vel_pid_ptr_; };
  std::shared_ptr<PidParams> getAnglePid() const { return angle_pid_ptr_; };
  std::shared_ptr<PidParams> getSpeedPid() const { return speed_pid_ptr_; };
  std::shared_ptr<PidParams> getDriveWheelSpeedPid() const { return drive_wheel_speed_pid_ptr_; };

 public:
  static BikePid& getInstance() {
    static BikePid instance;
    return instance;
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_pid_target_current_;
  std::shared_ptr<PidParams> angle_pid_ptr_;
  std::shared_ptr<PidParams> angle_vel_pid_ptr_;
  std::shared_ptr<PidParams> speed_pid_ptr_;
  std::shared_ptr<PidParams> drive_wheel_speed_pid_ptr_;
};

#endif  // BIKE_PID_HPP