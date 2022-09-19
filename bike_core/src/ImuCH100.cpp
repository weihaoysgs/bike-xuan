#include "bike_core/ImuCH100.hpp"

ImuCH100::ImuCH100() {
  imu_ch100_ser_port_ = std::make_shared<QSerialPort>();
  connect(imu_ch100_ser_port_.get(), SIGNAL(readyRead()), this,
          SLOT(ReadIMUCH100SerialDataCallback()));
  if (!InitIMUSerialPort("/dev/ttyUSB0", 115200)) exit(-1);
}

void ImuCH100::ReadIMUCH100SerialDataCallback() {
  LOG(INFO) << "Hello IMU Callback";
}

bool ImuCH100::InitIMUSerialPort(const std::string &port_name,
                                 const int baud_rate) {
  imu_ch100_ser_port_->setPortName(QString::fromStdString(port_name));
  imu_ch100_ser_port_->setBaudRate(baud_rate);
  imu_ch100_ser_port_->setDataBits(QSerialPort::Data8);
  imu_ch100_ser_port_->setParity(QSerialPort::EvenParity);
  imu_ch100_ser_port_->setStopBits(QSerialPort::OneStop);
  imu_ch100_ser_port_->setFlowControl(QSerialPort::NoFlowControl);

  if (imu_ch100_ser_port_->open(QIODevice::ReadWrite)) {
    LOG(INFO) << "Serial is open Success In : "
              << imu_ch100_ser_port_->portName().toStdString();
    imu_ch100_ser_port_->setBaudRate(baud_rate);
    imu_ch100_ser_port_->setDataBits(QSerialPort::Data8);
    imu_ch100_ser_port_->setParity(QSerialPort::NoParity);
    imu_ch100_ser_port_->setStopBits(QSerialPort::OneStop);
    imu_ch100_ser_port_->setFlowControl(QSerialPort::NoFlowControl);
    return true;
  } else {
    LOG(ERROR) << "Remote Ctl Serial"
               << imu_ch100_ser_port_->portName().toStdString()
               << " Open Failed";
    return false;
  }
}