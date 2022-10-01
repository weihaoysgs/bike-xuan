#ifndef REMOTE_CONTROL_PARSER_HPP
#define REMOTE_CONTROL_PARSER_HPP

#include <bike_core/remote_control_msg.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <QApplication>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <memory>
#include <thread>

class RemoteCtrlNode {
 public:
  RemoteCtrlNode(ros::NodeHandle &nh) : nh_(nh) {
    remote_ctrl_msg_pub_ =
        nh_.advertise<bike_core::remote_control_msg>("remote_ctrl_data", 1);
  }

  const ros::Publisher &GetRemoteCtrlMsgPublisher() const {
    return remote_ctrl_msg_pub_;
  }

 private:
  ros::Publisher remote_ctrl_msg_pub_;
  ros::NodeHandle nh_;
};

class RemoteControlDataParser : public QObject {
  Q_OBJECT
 public:
  RemoteControlDataParser();
  ~RemoteControlDataParser() override = default;

 public:
  void NodeSpinThread() {
    ros::spin();
    ros::shutdown();
  };
 public slots:
  void ReadRemoteControlSerialDataCallback();

 private:
  void Parser(const QByteArray &sbus_buf,
              bike_core::remote_control_msg &rc_ctrl) const;
  bool InitRemoteCtrlSerialport(const std::string &port_name) const;

 private:
  const int remote_ctrl_baud_rate_{100000};
  const int remote_ctrl_received_data_len_{18};

  std::shared_ptr<QSerialPort> remote_control_serial_;
  std::shared_ptr<RemoteCtrlNode> remote_ctl_node_;
};

#endif  // REMOTE_CONTROL_PARSER_HPP
