#include "bike_core/BikeXuanControl.hpp"
float target_speed_;
float current_speed;

float gyro_x_speed;
float roll_angle;
extern int start_pid;  // 动量轮开启标志位
extern float Tar_Ang_Vel_Y;

BikeXuanControl::BikeXuanControl() : nh_("~") {
  rc_ctrl_msg_ptr_ = std::make_shared<bike_core::remote_control_msg>();
  odrive_src_can_msg_ptr_ = std::make_shared<bike_core::odrive_can_msg>();
  bike_xuan_imu_msg_ptr_ = std::make_shared<sensor_msgs::Imu>();
  imu_ch100_pose_ptr_ = std::make_shared<ImuPose>();
  odrive_can_parsed_msg_ptr_ =
      std::make_shared<bike_core::odrive_motor_feedback_msg>();

  sub_pid_params_ = nh_.subscribe<bike_core::pid_params_msg>(
      "pid_debug_params", 100, &BikeXuanControl::SubPidParamsMessageCB, this);

  sub_can_src_msg_ = nh_.subscribe<bike_core::odrive_can_msg>(
      "topic1", 100, &BikeXuanControl::SubOdriveCanSourceMessageCB, this);
  sub_remote_ctrl_msg_ = nh_.subscribe<bike_core::remote_control_msg>(
      "/parser_remote_data_node/remote_ctrl_data", 10,
      &BikeXuanControl::SubRemoteControlMessageCB, this);
  sub_momentum_wheel_parsed_msg_ =
      nh_.subscribe<bike_core::odrive_motor_feedback_msg>(
          "/can_send_receive_node/odrive_motor_parsed_data", 10,
          &BikeXuanControl::SubOdriveMotorFeedbackParsedMessageCB, this);

  sub_imu_ch100_msg_ = nh_.subscribe<sensor_msgs::Imu>(
      "/IMU_data", 1, &BikeXuanControl::SubIMUCH100MessageCallback, this);

  bike_balance_timer_ = nh_.createTimer(ros::Duration(1.0 / 1000),
                                        &BikeXuanControl::timerBalance, this);

  std::thread t_bike_core_control =
      std::thread(&BikeXuanControl::tBikeCoreControl, this);
  t_bike_core_control.detach();

  std::thread t_bike_balance = std::thread(&BikeXuanControl::tBalance, this);
  t_bike_balance.detach();

  std::thread update = std::thread(&BikeXuanControl::tUpdate, this);
  update.detach();
}

void BikeXuanControl::tUpdate() {
  while (ros::ok()) {
    gyro_x_speed = bike_xuan_imu_msg_ptr_->angular_velocity.x;
    roll_angle = radian2angle(imu_ch100_pose_ptr_->roll_);
    current_speed = odrive_can_parsed_msg_ptr_->speed;
  }
}

int t_ms, t_2ms, t_10ms, t_100ms;
void BikeXuanControl::tBalance() {
  ros::Rate rate(10);  // hz
  while (ros::ok()) {
    if (start_pid == 1) {
      if (t_2ms == 1) {
        Balance_endocyclic();
        t_2ms = 0;
      }
      if (t_10ms == 1) {
        Balance_outcyclic();
        t_10ms = 0;
      }
      if (t_100ms == 1) {
        Speed_control();
        t_100ms = 0;
      }
    } else {
      target_speed_ = 0;
      Integral_clear();
    }

    // rate.sleep();
  }
}

void BikeXuanControl::timerBalance(const ros::TimerEvent &event) {
  t_ms++;
  if (t_ms % 2 == 0) t_2ms = 1;
  if (t_ms % 10 == 0) t_10ms = 1;
  if (t_ms % 100 == 0) {
    t_100ms = 1;
    t_ms = 0;
  }
}
extern PID Ang_Vel_PID;
extern PID Angle_PID;
extern PID MOTOR_PID;
extern PID Speed_PID;

void BikeXuanControl::SubPidParamsMessageCB(
    const bike_core::pid_params_msg::ConstPtr &msg) {
  if (msg->header.frame_id == "Angle.Vel.Pid")
  {
    Ang_Vel_PID.Kp = msg->p;
    Ang_Vel_PID.Ki = msg->i;
    Ang_Vel_PID.Kd = msg->d;
    LOG(INFO) << "Angle.Vel.Pid";
  }
  if (msg->header.frame_id == "Angle.Pid")
  {
    Angle_PID.Kp = msg->p;
    Angle_PID.Ki = msg->i;
    Angle_PID.Kd = msg->d;
    LOG(INFO) << "Angle.Pid";
  }
  if (msg->header.frame_id == "Motor.Pid")
  {
    MOTOR_PID.Kp = msg->p;
    MOTOR_PID.Ki = msg->i;
    MOTOR_PID.Kd = msg->d;
    LOG(INFO) << "Motor.Pid";
  }
  if (msg->header.frame_id == "Speed.Pid")
  {
    Speed_PID.Kp = msg->p;
    Speed_PID.Ki = msg->i;
    Speed_PID.Kd = msg->d;
    LOG(INFO) << "Speed.Pid";
  }
      
  LOG(INFO) << "msg.p: " << msg->p 
            << "\tmsg.i: " << msg->i
            << "\tmsg.d: " << msg->d;
  LOG(INFO) << "p: " << Ang_Vel_PID.Kp 
            << "\ti: " << Ang_Vel_PID.Ki
            << "\td: " << Ang_Vel_PID.Kd;
}

void BikeXuanControl::tBikeCoreControl() {
  const std::string can_port_name = "can0";
  const int socket_can_fd =
      CanSendReceive::GetOneSocketCanSendInstance(can_port_name.c_str());

  LOG_IF(FATAL, !socket_can_fd) << "Get Socket Can Instance Erros!";

  // TODO using yaml to save params
  const double control_rate = 1000;
  const double tolerance_msg_dt = 0.1;
  geometry_msgs::Vector3 &gyro_msg = bike_xuan_imu_msg_ptr_->angular_velocity;
  ros::Rate rate(control_rate);

  // radian2angle(imu_ch100_pose_ptr_->roll_)
  // gyro_msg.x

  constexpr double balance_roll_angle = 6.8;
  ros::Publisher pub_debug_plot = nh_.advertise<geometry_msgs::Vector3>("debug_plot", 1, this);
  geometry_msgs::Vector3 debug_plot_msg;
  BikePid bike_pid;

  while (ros::ok()) {
    // TODO set the target motor speed 0.0
    LOG_IF(FATAL, !ChechSubscriberMessageTimestamp())
        << "ChechSubscriberMessageTimestamp Failed!\t" <<
        []() -> std::string { return std::string("Set Motor Speed To [0.0]"); };

    float target_remote_speed = rc_ctrl_msg_ptr_->ch_x[0] / 5.0;

    if (rc_ctrl_msg_ptr_->s1 == 1) {
      start_pid = 1;  // 开
    } else {
      start_pid = 0;
    }

    LOG_IF(WARNING, 1) << std::setprecision(4) << std::setiosflags(std::ios::fixed) 
                     << setiosflags(std::ios::showpos) <<
                       "current_speed: " << current_speed << 
                        "\tgyro_x_speed:" << gyro_x_speed << 
                        "\tzero_error:" << roll_angle -  6.8 << 
                        "\ttarget_speed_: " << target_speed_;

    int16_t int_speed = static_cast<int16_t>(target_speed_);
    debug_plot_msg.x = -gyro_x_speed;
    debug_plot_msg.y = Tar_Ang_Vel_Y;
    pub_debug_plot.publish(debug_plot_msg);
    CanSendReceive::WriteDataToSocketCanDeviceControlMotor(socket_can_fd, 524,
                                                           int_speed);
    rate.sleep();
  }
  close(socket_can_fd);
}

const bool BikeXuanControl::ChechSubscriberMessageTimestamp() const {
  double time_now = ros::Time::now().toSec();
  double remote_ctrl_data_time = rc_ctrl_msg_ptr_->header.stamp.toSec();
  double motor_parsed_data_time =
      odrive_can_parsed_msg_ptr_->header.stamp.toSec();
  LOG_IF(WARNING, 0) << "dt remote_ctrl_data_time: "
                     << std::abs(time_now - remote_ctrl_data_time)
                     << "  dt motor_parsed_data_time: "
                     << std::abs(time_now - motor_parsed_data_time);
  return true;
}