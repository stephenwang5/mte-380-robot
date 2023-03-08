#include "test.h"

#include "main.h"

void turnInPlace(int degrees) {

  bool clockwise = degrees > 0;
  constexpr int pwm = 100;

  float turnDist = PI*12*(abs(degrees))/360.0f; // turn distance of each wheel
  uint16_t numticks = uint16_t((turnDist*TICKS_PER_REV)/(PI*2*wheelRadius)); // number of encoder ticks needed to turn provided number of degrees

  const int R_startPos = rightMotor.encoder;
  const int L_startPos = leftMotor.encoder;

  if (clockwise) { // Left wheel goes forward, right wheel goes back => both wheels spin CCW
    rightMotor.rotateCCW(pwm);
    leftMotor.rotateCCW(pwm);

  } else { // Right wheel goes forward, left wheel goes back => both wheels spin CW
    rightMotor.rotateCW(pwm);
    leftMotor.rotateCW(pwm);
  }

  bool Rturning  = 1, Lturning = 1;
  while (Rturning || Lturning) {
    if (abs(rightMotor.encoder - R_startPos) >= numticks) {
      rightMotor.coast();
      Rturning = 0;
    }
    if (abs(leftMotor.encoder - L_startPos) >= numticks) {
      leftMotor.coast();
      Lturning = 0;
    }
  }

}

// FIXME: only exists for unit testing. Not thread friendly
void turnToMagPos(int magX, int magY){

  readIMU();
  int currentMagX = imu.mx;
  int currentMagY = imu.my;
  int pwm = 50;
  rightMotor.rotateCW(pwm);
  leftMotor.rotateCW(pwm);
  while((currentMagX > 1.05*magX || currentMagX < 0.95*magX) && (currentMagY > 1.05*magY || currentMagY < 0.95*magY)){
    readIMU();
    currentMagX = imu.mx;
    currentMagY = imu.my;
  }

  coast();
}

// FIXME: only exists for unit testing. Not thread friendly
void turnToYawPos(int yaw) {

  readIMU();
  int currentYaw = imu.yaw;
  int pwm = 50;
  rightMotor.rotateCW(pwm);
  leftMotor.rotateCW(pwm);

  while((currentYaw > 1.05*yaw || currentYaw < 0.95*yaw)){
    readIMU();
    currentYaw = imu.yaw;
  }

  coast();
}

void rampUpBothMotors() {
  static uint8_t pwm = 0;
  while (1) {
    leftMotor.rotateCCW(pwm >> 2);
    rightMotor.rotateCW(pwm++ >> 2);
    rtos::ThisThread::sleep_for(10ms);
  }
}

void motorDemo() {
  turnInPlace(-90);
  delay(2000);
  forward(100);
  delay(2000);
  coast();
  delay(2000);
  turnInPlace(90);
  delay(2000);
  backward(100);
  delay(2000);
  coast();
  delay(4000);
}