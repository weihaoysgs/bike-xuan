#ifndef IMUCH100_HPP
#define IMUCH100_HPP

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <memory>

class ImuCH100 : public QObject {
  Q_OBJECT
 public:
  ImuCH100();
  ~ImuCH100() override = default;
 public slots:

  void ReadIMUCH100SerialDataCallback();

 private:
  bool InitIMUSerialPort(const std::string &port_name, const int baud_rate);

 private:
  std::shared_ptr<QSerialPort> imu_ch100_ser_port_;
};

#endif  // IMUCH100_HPP
