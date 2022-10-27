#include "bike_core/ServoControl.hpp"

ServoControl::ServoControl() {
  servo_protocol_ptr_ = std::make_shared<fsuservo::FSUS_Protocol>(
      OdriveMotorConfig::getSigleInstance().servo_port_name_,
      static_cast<itas109::BaudRate>(
          OdriveMotorConfig::getSigleInstance().servo_port_baud_rate_));
  servo0_ptr_ = std::make_shared<fsuservo::FSUS_Servo>(
      OdriveMotorConfig::getSigleInstance().servo_id_,
      servo_protocol_ptr_.get());
  if (servo0_ptr_->ping())
    LOG(INFO) << "Servo : " << OdriveMotorConfig::getSigleInstance().servo_id_
              << " is online;";
  else
    LOG(FATAL) << "Servo is Offline!!!";
}

void ServoControl::SetSerilaServoAngle(const float angle) {
  static std::function<void(fsuservo::FSUS_Protocol&, fsuservo::FSUS_Servo*)>
      wait_servo_report = [](fsuservo::FSUS_Protocol& protocol,
                             fsuservo::FSUS_Servo* servo) -> bool {
    servo->wait();
    LOG(INFO) << "Real Angle = " << static_cast<float>(servo->curRawAngle)
              << ", Target Angle = " << servo->targetRawAngle << endl;
    protocol.delay_ms(20);
  };
  servo0_ptr_->setRawAngle(angle);
  wait_servo_report(*servo_protocol_ptr_.get(), servo0_ptr_.get());
}
