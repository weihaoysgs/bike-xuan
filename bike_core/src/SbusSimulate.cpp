#include "bike_core/SbusSimulate.hpp"

SbusSimulateSerial::SbusSimulateSerial() {
  LOG(INFO) << "Hello SbusSimulateSerial";
  sbus_simulate_ser_ = std::make_shared<QSerialPort>();
  InitSbusSimulateSerialPort(
      OdriveMotorConfig::getSigleInstance().servo_port_name_);
}

bool SbusSimulateSerial::InitSbusSimulateSerialPort(
    const std::string port_name) const {
  sbus_simulate_ser_->setPortName(QString::fromStdString(port_name));
  sbus_simulate_ser_->setBaudRate(100000);
  sbus_simulate_ser_->setDataBits(QSerialPort::Data8);
  sbus_simulate_ser_->setParity(QSerialPort::EvenParity);
  sbus_simulate_ser_->setStopBits(QSerialPort::TwoStop);
  sbus_simulate_ser_->setFlowControl(QSerialPort::NoFlowControl);
  if (sbus_simulate_ser_->open(QIODevice::ReadWrite)) {
    LOG(INFO) << "Open SBUS Simulate Serial Success in: "
              << sbus_simulate_ser_->portName().toStdString();
    return true;

  } else {
    LOG(FATAL) << "Open SBUS Simulate Serial Failed In: " << port_name;
    return false;
  }
}
