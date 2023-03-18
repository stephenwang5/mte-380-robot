#include "control.h"

#include "main.h"

// Variables for general PID used to match encoder ticks from both motors
double pid_setpoint, pid_output, pid_input;
double Kp=1, Ki=0.1, Kd=0;
PID pidController(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
uint8_t target_pwm = 50; //0-255
uint8_t leftpwm, rightpwm;

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
