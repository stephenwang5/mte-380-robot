#ifndef MOTOR_H
#define MOTOR_H

#include <PID_v1.h>

enum Direction {
  CW = 1,
  CCW = -1,
};

class Motor{
public:

  int64_t encoder;
  bool encMem;
  static constexpr uint8_t encBufLength = 4;
  int64_t encBuf[encBufLength];
  int64_t encBufTime[encBufLength];
  uint8_t encBufIdx;
  bool encBufInitialized;
  double speed;

  const PinName pinA;
  const PinName pinB;
  const PinName encoderPin;
  Direction direction;

  // TODO: if the PID gain terms aren't dynamic, make them constexpr
  double Kp=40, Ki=1.0, Kd=1.0;
  double lastEncoderCount = 0;

  Motor(PinName a, PinName b, PinName enc);
  void begin();
  void rotateCW(uint8_t);
  void rotateCCW(uint8_t);
  void coast();
  void activeBreak();

  void encoderUpdate();
  void calculateSpeed();
  void controlSpeed();
  double Setpoint, Output;

  PID pidController; // Leaving this here as we decide which PID to use
};

// Variables for general PID used to match encoder ticks from both motors
extern double pid_setpoint, pid_output, pid_input;
extern double Kp, Ki, Kd;
extern PID straightDrivePID;
constexpr uint8_t pwm_straight_drive = 55; //0-255
constexpr float leftFactor = 1.0;
constexpr float rightFactor = 1.2;
extern bool pole_located;
extern bool survey_timeout;
extern float avgHeading;

extern uint8_t leftpwm, rightpwm;
// int acc_error;
// double P;
// double I;
// double D;
// int prev_error;

void spinCCW(uint8_t pwmL, uint8_t pwmR);
void spinCW(uint8_t pwmL, uint8_t pwmR);
void registerEncoderISRs();
void forward(uint8_t pwmL, uint8_t pwmR);
void backward(uint8_t pwmL, uint8_t pwmR);
void coast();
void activeBreak();
void calculateMotorSpeeds();
void controlMotorSpeedsForTurning();
void TurnInPlaceByNumDegrees(float degrees);
void driveStraight();
void turnInPlaceByMag(float targetMag, uint8_t pwm);
void driveToPole();

#endif // MOTOR_H
