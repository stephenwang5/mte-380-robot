#include "control.h"

#include "main.h"

using namespace std::chrono_literals;

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
