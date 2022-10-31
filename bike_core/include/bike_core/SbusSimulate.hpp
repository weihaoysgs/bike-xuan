#ifndef SBUS_SIMULATE_HPP
#define SBUS_SIMULATE_HPP

#include <glog/logging.h>
#include <qserialport.h>
#include <ros/ros.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <thread>

#include "bike_core/OdriveMotorConfig.hpp"

#define SBUS_FRAME_SIZE 25
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_SCALE_FACTOR                \
  ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / \
   (SBUS_RANGE_MAX - SBUS_RANGE_MIN))  // 0.625f
#define SBUS_SCALE_OFFSET \
  (int)(SBUS_TARGET_MIN - \
        (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))  // 874.5f

class SbusSimulateSerial {
 public:
  SbusSimulateSerial();
  ~SbusSimulateSerial() = default;
  bool InitSbusSimulateSerialPort(const std::string port_name) const;
  const qint16 SbusSimulateOutput(const uint16_t channels_num) const;
  void SetOutputValues(const std::array<uint16_t, 16> &values);

 private:
  void SbusOutputThread();

 private:
  std::array<uint16_t, 16> values_{0};
  std::shared_ptr<QSerialPort> sbus_simulate_ser_;
};

#endif  // SBUS_SIMULATE_HPP