#include "control.h"
#include "motor.h"
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