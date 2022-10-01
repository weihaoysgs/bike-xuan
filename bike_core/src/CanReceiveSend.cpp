#include "bike_core/CanReceiveSend.hpp"

CanSendReceive::CanSendReceive() : nh_("~") {
  receive_can_port_name_ = "can0";
  pub_can_msg_ =
      nh_.advertise<bike_core::odrive_can_msg>("odrive_src_can_msg", 100);

  std::thread t_receive_odrive_can_msg_pub =
      std::thread(&CanSendReceive::tReceivePublishCanMsg, this);
  t_receive_odrive_can_msg_pub.detach();

  // std::thread t_send_special_cmd =
  //     std::thread(&CanSendReceive::tSendSpecialCommand, this);
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
  (void)bind(socket_can, (struct sockaddr *)&addr,
             sizeof(addr));  // bind the socket and canx
  LOG_IF(FATAL, socket_can == -1)
      << "Socket Can " + can_port_name + " Open Failed ";
  return socket_can;
}

void CanSendReceive::tSendSpecialCommand() const {
  int socket_can_fd = GetOneSocketCanInstance(receive_can_port_name_);
  std::array<uint8_t, 8> data;
  data.at(0) = 0x10;
  // cansend can0 209#r1000000000000000 查询电机当前的速度
  // cansend can0 20C#0000000007D00000 给点击速度指令
  while (ros::ok()) {
    WriteDataToSocketCanDevice(socket_can_fd, 209, data);
  }
}

int CanSendReceive::WriteDataToSocketCanDevice(const int &socket_can_fd,
                                               const canid_t &can_id,
                                               std::array<uint8_t, 8> &data) {
  struct can_frame can_send_frame {};
  can_send_frame.can_id = can_id;
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
  constexpr uint32_t motor_position_vel_can_msg_can_id = 521;
  if (motor_position_vel_can_msg_can_id == can_id) {
    uint8_t data[8];
    data[0] = receive_can_frame.data[0];
    data[1] = receive_can_frame.data[1];
    data[2] = receive_can_frame.data[2];
    data[3] = receive_can_frame.data[3];
    data[4] = receive_can_frame.data[4];
    data[5] = receive_can_frame.data[5];
    data[6] = receive_can_frame.data[6];
    data[7] = receive_can_frame.data[7];
    float *position = (float *)data;
    float *speed = (float *)(data + 4);
    std::cout << std::hex << "Data: " << data[0] << " " << data[1] << " " << data[2] << " "
              << data[3] << " " << data[4] << " " << data[5] << " " << data[6]
              << " " << data[7];
    LOG(INFO) << std::dec << "Speed: " << *speed << " "
              << "Position: " << *position;
  }
}