#include "motor.h"

void Motor::clockwise(uint8_t pwm) {
  analogWrite(PinA, pwm);
  analogWrite(PinB, 0);
  direction = CW;
}

void Motor::counterclockwise(uint8_t pwm) {
  analogWrite(PinA, 0);
  analogWrite(PinB, pwm);
  direction = CCW;
}

void Motor::coast() {
  analogWrite(PinA, 0);
  analogWrite(PinB, 0);
}

// uint8_t Motor::controlSpeed() {

//   encoder.speed = (encoder.encoderPos - lastEncoderCount) * 1000 / pidController.GetSampleTime();
//   lastEncoderCount = encoder.encoderPos;

//   //Run the PID calculation
//   pidController.Compute();

//   return Output;
// }

void Motor::setSetpoint(uint8_t setpoint){
  Setpoint = setpoint;
}
