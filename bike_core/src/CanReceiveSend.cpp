#include "bike_core/CanReceiveSend.hpp"

CanSendReceive::CanSendReceive() : nh_("~") {
  receive_can_port_name_ = "can0";
  pub_can_msg_ =
      nh_.advertise<bike_core::odrive_can_msg>("odrive_src_can_msg", 100);
  pub_odrive_motor_msg_ = nh_.advertise<bike_core::odrive_motor_feedback_msg>(
      "odrive_motor_parsed_data", 1);
  std::thread t_receive_odrive_can_msg_pub =
      std::thread(&CanSendReceive::tReceivePublishCanMsg, this);
  t_receive_odrive_can_msg_pub.detach();
  std::thread t_send_special_cmd =
      std::thread(&CanSendReceive::tSendSpecialCommand, this);
  t_send_special_cmd.detach();
}

void CanSendReceive::tReceivePublishCanMsg() const {
  int socket_can_fd = GetOneSocketCanInstance(receive_can_port_name_);
  struct can_frame receive_can_frame {};
  while (ros::ok()) {
    ssize_t n_bytes =
        read(socket_can_fd, &receive_can_frame, sizeof(receive_can_frame));
    if (n_bytes == 16) {
      bike_core::odrive_can_msg can_rxd_msg;
      can_rxd_msg.header.stamp = ros::Time::now();
      can_rxd_msg.bytes[0] = receive_can_frame.data[0];
      can_rxd_msg.bytes[1] = receive_can_frame.data[1];
      can_rxd_msg.bytes[2] = receive_can_frame.data[2];
      can_rxd_msg.bytes[3] = receive_can_frame.data[3];
      can_rxd_msg.bytes[4] = receive_can_frame.data[4];
      can_rxd_msg.bytes[5] = receive_can_frame.data[5];
      can_rxd_msg.bytes[6] = receive_can_frame.data[6];
      can_rxd_msg.bytes[7] = receive_can_frame.data[7];
      can_rxd_msg.can_id = receive_can_frame.can_id;
      can_rxd_msg.data_len = receive_can_frame.can_dlc;
      ParserSpecialCanMessage(receive_can_frame.can_id, receive_can_frame);
      pub_can_msg_.publish(can_rxd_msg);
    } else {
      LOG(ERROR) << "Read Can Msg Error Receive Bytes Is Not 16";
    }
  }
  close(socket_can_fd);
}

int CanSendReceive::GetOneSocketCanInstance(const std::string &can_port_name) {
  struct sockaddr_can addr {};
  struct ifreq ifr {};
  // create can socket
  int socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr.ifr_name, can_port_name.c_str());
  // specified the can device
  ioctl(socket_can, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  int bind_ret = bind(socket_can, (struct sockaddr *)&addr,
                      sizeof(addr));  // bind the socket and canx
  LOG_IF(FATAL, socket_can == -1 || bind_ret < 0)
      << "Socket Can " + can_port_name + " Open Failed ";
  return socket_can;
}

void CanSendReceive::tSendSpecialCommand() const {
  int socket_can_fd = GetOneSocketCanInstance(receive_can_port_name_);
  const std::tuple<const canid_t, const bool, const std::array<uint8_t, 8>>
      check_axis0_vel_position_cmd{OdriveMotorConfig::getSigleInstance()
                                       .axis0_send_receive_vel_position_can_id_,
                                   true,
                                   {0x10}};

  const std::tuple<const canid_t, const bool, const std::array<uint8_t, 8>>
      check_axis1_vel_position_cmd{OdriveMotorConfig::getSigleInstance()
                                       .axis1_send_receive_vel_position_can_id_,
                                   true,
                                   {0x10}};

  ros::Rate rate(100);
  while (ros::ok()) {
    if (!WriteDataToSocketCanDevice(
            socket_can_fd,
            std::get<const canid_t>(check_axis0_vel_position_cmd),
            std::get<const bool>(check_axis0_vel_position_cmd),
            std::get<const std::array<uint8_t, 8>>(
                check_axis0_vel_position_cmd))) {
      LOG(ERROR) << "Send Axis0 Can Message Error";
    }

    if (!WriteDataToSocketCanDevice(
            socket_can_fd,
            std::get<const canid_t>(check_axis1_vel_position_cmd),
            std::get<const bool>(check_axis1_vel_position_cmd),
            std::get<const std::array<uint8_t, 8>>(
                check_axis1_vel_position_cmd))) {
      LOG(ERROR) << "Send Axis1 Can Message Error";
    }

    rate.sleep();
  }
}

int CanSendReceive::WriteDataToSocketCanDevice(
    const int &socket_can_fd, const canid_t &can_id, const bool is_RTR,
    const std::array<uint8_t, 8> &data) {
  struct can_frame can_send_frame {};
  can_send_frame.can_id = can_id;
  if (is_RTR) can_send_frame.can_id |= CAN_RTR_FLAG;
  can_send_frame.data[0] = data.at(0);
  can_send_frame.data[1] = data.at(1);
  can_send_frame.data[2] = data.at(2);
  can_send_frame.data[3] = data.at(3);
  can_send_frame.data[4] = data.at(4);
  can_send_frame.data[5] = data.at(5);
  can_send_frame.data[6] = data.at(6);
  can_send_frame.data[7] = data.at(7);

  ssize_t n_bytes =
      write(socket_can_fd, &can_send_frame, sizeof(can_send_frame));
  LOG_IF(ERROR, n_bytes == -1) << "Write Scoket Can Failed";
  return n_bytes;
}

void CanSendReceive::ParserSpecialCanMessage(
    uint32_t can_id, can_frame &receive_can_frame) const {
  // TODO: change the ID define position
  uint32_t axisx_motor_position_vel_can_msg_can_id = can_id;
  if (can_id == OdriveMotorConfig::getSigleInstance()
                    .axis0_send_receive_vel_position_can_id_ ||
      can_id == OdriveMotorConfig::getSigleInstance()
                    .axis1_send_receive_vel_position_can_id_) {
    float *position = reinterpret_cast<float *>(receive_can_frame.data);
    float *speed = reinterpret_cast<float *>(receive_can_frame.data + 4);
    bike_core::odrive_motor_feedback_msg odrive_motor_vel_position_data;
    odrive_motor_vel_position_data.header.stamp = ros::Time::now();
    odrive_motor_vel_position_data.position = *position;
    odrive_motor_vel_position_data.speed = *speed;
    odrive_motor_vel_position_data.can_id =
        axisx_motor_position_vel_can_msg_can_id;
    pub_odrive_motor_msg_.publish(odrive_motor_vel_position_data);
    // LOG(INFO) << std::dec << "Speed: " << *speed << " "
    //           << "Position: " << *position;
  }
}

int CanSendReceive::GetOneSocketCanSendInstance(const char *port_name) {
  int s;
  struct sockaddr_can addr;
  struct ifreq ifr;

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    LOG(FATAL) << "Socket Can Get Error";
    return -1;
  }

  strncpy(ifr.ifr_name, port_name, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    LOG(FATAL) << "if_nametoindex";
    return -1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    LOG(FATAL) << "Bind Socket Can Error";
    return -1;
  }
  return s;
}

int CanSendReceive::WriteDataToSocketCanDeviceControlMotor(
    const int &socket_can_fd, const canid_t &can_id, const int16_t int_speed) {
  can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  // frame.data[4] = int_speed >> 8;
  // frame.data[5] = int_speed;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = int_speed;
  frame.data[7] = int_speed >> 8;

  int n = write(socket_can_fd, &frame, sizeof(frame));
  LOG_IF(ERROR, n == -1) << "Send Error";
}