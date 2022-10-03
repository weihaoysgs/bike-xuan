#include "bike_core/BikeXuanControl.hpp"

int GetOneSocketCanSendInstance(const char *port_name)
{
	int s; 
	int required_mtu;
	int mtu;
	int enable_canfd = 1;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		printf("socket\n");
		return 1;
	}

	strncpy(ifr.ifr_name, port_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		printf("if_nametoindex\n");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printf("wirete err\n");
		return 1;
	}
	return s;
}

BikeXuanControl::BikeXuanControl() : nh_("~") {
  rc_ctrl_msg_ptr_ = std::make_shared<bike_core::remote_control_msg>();
  odrive_src_can_msg_ptr_ = std::make_shared<bike_core::odrive_can_msg>();
  odrive_can_parsed_msg_ptr_ =
      std::make_shared<bike_core::odrive_motor_feedback_msg>();
  sub_can_src_msg_ = nh_.subscribe<bike_core::odrive_can_msg>(
      "topic1", 100, &BikeXuanControl::SubOdriveCanSourceMessageCB, this);
  sub_remote_ctrl_msg_ = nh_.subscribe<bike_core::remote_control_msg>(
      "/parser_remote_data_node/remote_ctrl_data", 10,
      &BikeXuanControl::SubRemoteControlMessageCB, this);
  sub_momentum_wheel_parsed_msg_ =
      nh_.subscribe<bike_core::odrive_motor_feedback_msg>(
          "/can_send_receive_node/odrive_motor_parsed_data", 10,
          &BikeXuanControl::SubOdriveMotorFeedbackParsedMessageCB, this);
  std::thread t_bike_core_control =
      std::thread(&BikeXuanControl::tBikeCoreControl, this);
  t_bike_core_control.detach();
}

void BikeXuanControl::tBikeCoreControl() {
  const std::string can_port_name = "can0";
  const int socket_can_fd = GetOneSocketCanSendInstance(can_port_name.c_str());
    //   CanSendReceive::GetOneSocketCanInstance(can_port_name);
  LOG_IF(FATAL, !socket_can_fd) << "Get Socket Can Instance Erros!";
  
  // TODO using yaml to save params
  const double control_rate = 1000;
  const double tolerance_msg_dt = 0.1;

  ros::Rate rate(control_rate);

  auto set_motor_speed = [this](const int &socket_can_fd, const canid_t &can_id,
                                can_frame &frame,
                                float speed) -> bool {
    int16_t int_speed = static_cast<int16_t>(speed);

    frame.data[4] = int_speed >> 8;
    frame.data[5] = int_speed;
    int n = write(socket_can_fd, &frame, sizeof(frame));
        LOG_IF(ERROR, n==-1) << "Send Error";
  };
  std::array<uint8_t, 8> data;

  while (ros::ok()) {
    // TODO 同时也应该将电机速度赋值为 0
    LOG_IF(FATAL, !ChechSubscriberMessageTimestamp())
        << "ChechSubscriberMessageTimestamp Failed!\t" <<
        []() -> std::string { return std::string("Set Motor Speed To [0.0]"); };

    float target_speed = rc_ctrl_msg_ptr_->ch_x[0] / 5.0;
    float current_speed = odrive_can_parsed_msg_ptr_->speed;
    float error = target_speed - current_speed;
    float kp = 1.0;
    float ki = 1.0;
    float kd = 1.0;
    
    
    int16_t int_speed = static_cast<int16_t>(error*kp);

    LOG_IF(WARNING, 1) << "current_speed: " << current_speed << "\t"
                       << "target_speed: " << target_speed << "\terror: " << error << "\tint speed:" << int_speed;

    if (int_speed > 32760 || int_speed < -32760)
        int_speed = 32760;
    can_frame frame;
    frame.can_id = 524;
    frame.can_dlc = 8;
    frame.data[0] = 0x00;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = int_speed >> 8;
    frame.data[5] = int_speed;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;


    int n = write(socket_can_fd, &frame, sizeof(frame));
        LOG_IF(ERROR, n==-1) << "Send Error";
    // set_motor_speed(socket_can_fd, 524, frame, kp * error);
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