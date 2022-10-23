#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

#include <glog/logging.h>
#include <ros/ros.h>

#include <memory>

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "bike_core/OdriveMotorConfig.hpp"

class ServoControl {
 public:
  explicit ServoControl();
  ~ServoControl() = default;
  void SetSerilaServoAngle(const float angle);

 public:
  // C++ Single Mode
  static ServoControl& getSingleInstance(void) {
    static ServoControl servo_control;
    return servo_control;
  }

 private:
  std::shared_ptr<fsuservo::FSUS_Protocol> servo_protocol_ptr_;
  std::shared_ptr<fsuservo::FSUS_Servo> servo0_ptr_;
};

#endif  // SERVO_CONTROL_HPP
