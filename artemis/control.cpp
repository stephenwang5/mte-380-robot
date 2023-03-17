#include "control.h"

#include "main.h"

void spinCCW(uint8_t pwm) {
  if (orientation == IMU_FACE_UP) {
    leftMotor.rotateCW(pwm);
    rightMotor.rotateCW(pwm);
  } else {
    leftMotor.rotateCCW(pwm);
    rightMotor.rotateCCW(pwm);
  }
}

void spinCW(uint8_t pwm) {
  if (orientation == IMU_FACE_UP) {
    leftMotor.rotateCCW(pwm);
    rightMotor.rotateCCW(pwm);
  } else {
    leftMotor.rotateCW(pwm);
    rightMotor.rotateCW(pwm);
  }
}
