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
  double Kp=10, Ki=15, Kd=70;
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

void registerEncoderISRs();
void forward(uint8_t, uint8_t);
void backward(uint8_t, uint8_t);
void coast();
void activeBreak();
void calculateMotorSpeeds();
void controlMotorSpeeds();
void controlMotorSpeedsWithEncoderCount();


#endif // MOTOR_H
