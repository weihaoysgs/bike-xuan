#include "bike_core/RemoteControlParser.hpp"

RemoteControlDataParser::RemoteControlDataParser() {
  ros::NodeHandle nh("~");
  remote_ctl_node_ = std::make_shared<RemoteCtrlNode>(nh);
  remote_control_serial_ = std::make_shared<QSerialPort>();
  this->InitRemoteCtrlSerialport(OdriveMotorConfig::getSigleInstance().dbus_serial_port_name_);
  connect(remote_control_serial_.get(), SIGNAL(readyRead()), this,
          SLOT(ReadRemoteControlSerialDataCallback()));

  // launch the ros spin thread, which is in the qt main loop
  std::thread remote_ctl_thread =
      std::thread(&RemoteControlDataParser::NodeSpinThread, this);
  remote_ctl_thread.detach();
}

void RemoteControlDataParser::ReadRemoteControlSerialDataCallback() {
  qint64 rece_len = remote_control_serial_->bytesAvailable();
  QByteArray received_buffer = remote_control_serial_->readAll();
  if (!received_buffer.isEmpty() &&
      rece_len == remote_ctrl_received_data_len_) {
    bike_core::remote_control_msg remote_ctl_msg;
    remote_ctl_msg.header.stamp = ros::Time::now();
    Parser(received_buffer, remote_ctl_msg);
    remote_ctl_node_->GetRemoteCtrlMsgPublisher().publish(remote_ctl_msg);
  } else {
    LOG(ERROR) << "Remote Data Is Empty";
  }
}

bool RemoteControlDataParser::InitRemoteCtrlSerialport(
    const std::string &port_name) const {
  // remember to set the serial parameters, then open the device
  auto open_device = [this, port_name]() -> void {
    remote_control_serial_->setPortName(QString::fromStdString(port_name));
    remote_control_serial_->setBaudRate(remote_ctrl_baud_rate_);
    remote_control_serial_->setDataBits(QSerialPort::Data8);
    remote_control_serial_->setParity(QSerialPort::EvenParity);
    remote_control_serial_->setStopBits(QSerialPort::OneStop);
    remote_control_serial_->setFlowControl(QSerialPort::NoFlowControl);
  };
  open_device();
  if (remote_control_serial_->open(QIODevice::ReadWrite)) {
    LOG(INFO) << "Serial is open Success In : "
              << remote_control_serial_->portName().toStdString();
    open_device();
    return true;
  } else {
    LOG(ERROR) << "Remote Ctl Serial"
               << remote_control_serial_->portName().toStdString()
               << " Open Failed";
    return false;
  }
}

void RemoteControlDataParser::Parser(
    const QByteArray &sbus_buf, bike_core::remote_control_msg &rc_ctrl) const {
  auto sbus_buf_temp = reinterpret_cast<unsigned char *>(const_cast<char *>(sbus_buf.data()));
  rc_ctrl.ch_x[0] = static_cast<int16_t>((sbus_buf_temp[0] | (sbus_buf_temp[1] << 8)) & 0x07ff);         // Channel 0
  rc_ctrl.ch_x[1] = static_cast<int16_t>(((sbus_buf_temp[1] >> 3) | (sbus_buf_temp[2] << 5)) & 0x07ff);  // Channel 1
  rc_ctrl.ch_x[2] = static_cast<int16_t>(((sbus_buf_temp[2] >> 6) | (sbus_buf_temp[3] << 2) |
                            (sbus_buf_temp[4] << 10)) & 0x07ff);                                         // Channel 2
  rc_ctrl.ch_x[3] = static_cast<int16_t>(((sbus_buf_temp[4] >> 1) | (sbus_buf_temp[5] << 7)) & 0x07ff);  // Channel 3
  rc_ctrl.s1 = ((sbus_buf_temp[5] >> 4) & 0x0003);       // Switch left
  rc_ctrl.s2 = ((sbus_buf_temp[5] >> 4) & 0x000C) >> 2;  // Switch right
  rc_ctrl.mouse_x = static_cast<int16_t>( sbus_buf_temp[6] | (sbus_buf_temp[7] << 8));    // Mouse X axis
  rc_ctrl.mouse_y = static_cast<int16_t>( sbus_buf_temp[8] | (sbus_buf_temp[9] << 8));    // Mouse Y axis
  rc_ctrl.mouse_z = static_cast<int16_t>( sbus_buf_temp[10] | (sbus_buf_temp[11] << 8));  // Mouse Z axis
  rc_ctrl.mouse_press_left = sbus_buf_temp[12];       // Mouse Left Is Press ?
  rc_ctrl.mouse_press_right = sbus_buf_temp[13];      // Mouse Right Is Press ?
  rc_ctrl.key_value = sbus_buf_temp[14] | (sbus_buf_temp[15] << 8);  // KeyBoard value
  rc_ctrl.ch_x[4] = static_cast<int16_t>(sbus_buf_temp[16] | (sbus_buf_temp[17] << 8));  // NULL
  constexpr int RC_CH_VALUE_OFFSET = 1024;
  std::for_each(rc_ctrl.ch_x.begin(), rc_ctrl.ch_x.end(), [](int16_t &element) { element -= RC_CH_VALUE_OFFSET; });
}
