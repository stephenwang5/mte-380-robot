#include "motor.h"

#include "main.h"

Motor::Motor(PinName a, PinName b, PinName enc): pinA(a), pinB(b), encoderPin(enc) {}

void Motor::begin() {
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(encoderPin, INPUT);
}

void Motor::rotateCW(uint8_t pwm) {
  analogWrite(pinA, pwm);
  analogWrite(pinB, 0);
  direction = CW;
}

void Motor::rotateCCW(uint8_t pwm) {
  analogWrite(pinA, 0);
  analogWrite(pinB, pwm);
  direction = CCW;
}

void Motor::coast() {
  analogWrite(pinA, 0);
  analogWrite(pinB, 0);
}

void Motor::activeBreak() {
  analogWrite(pinA, 255);
  analogWrite(pinB, 255);
}

// uint8_t Motor::controlSpeed() {

//   encoder.speed = (encoder.encoderPos - lastEncoderCount) * 1000 / pidController.GetSampleTime();
//   lastEncoderCount = encoder.encoderPos;

//   //Run the PID calculation
//   pidController.Compute();

//   return Output;
// }

// void Motor::setSetpoint(uint8_t setpoint){
//   Setpoint = setpoint;
// }

void Motor::encoderUpdate() {
  encoder += direction;
}

void Motor::calculateSpeed() {
  int32_t curTime = micros();
  float duration = (curTime - prevTime) * 1e-6f;
  speed = (float)(encoder - prevEncoder) / TICKS_PER_REV / duration;

  prevTime = curTime;
  prevEncoder = encoder;
}

static void updateRightEncoder() {
  rightMotor.encoderUpdate();
}

static void updateLeftEncoder() {
  leftMotor.encoderUpdate();
}

void registerEncoderISRs() {
  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPin), updateRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPin), updateLeftEncoder, RISING);
}

void forward(uint8_t pwm){
  leftMotor.rotateCCW(pwm);
  rightMotor.rotateCW(pwm);
}

void backward(uint8_t pwm){
  leftMotor.rotateCW(pwm);
  rightMotor.rotateCCW(pwm);
}

void coast(){
  leftMotor.coast();
  rightMotor.coast();
}

void activeBreak() {
  leftMotor.activeBreak();
  rightMotor.activeBreak();
}
