#ifndef SBUS_SIMULATE_HPP
#define SBUS_SIMULATE_HPP

#include <glog/logging.h>
#include <qserialport.h>
#include <ros/ros.h>

#include <memory>

#include "bike_core/OdriveMotorConfig.hpp"

class SbusSimulateSerial {
 public:
  SbusSimulateSerial();
  ~SbusSimulateSerial() = default;
  bool InitSbusSimulateSerialPort(const std::string port_name) const;

 private:
  std::shared_ptr<QSerialPort> sbus_simulate_ser_;
};

#endif  // SBUS_SIMULATE_HPP