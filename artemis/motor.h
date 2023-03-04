#ifndef MOTOR_H
#define MOTOR_H

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
  float speed;

  const PinName pinA;
  const PinName pinB;
  const PinName encoderPin;
  Direction direction;

  // TODO: if the PID gain terms aren't dynamic, make them constexpr
  // double Kp=1, Ki=0.1, Kd=0.01;
  // double Setpoint, Input, Output;
  // double lastEncoderCount = 0;
  // PID pidController;

  Motor(PinName a, PinName b, PinName enc);
  void begin();
  void rotateCW(uint8_t);
  void rotateCCW(uint8_t);
  void coast();
  void activeBreak();
  //void controlSpeed();

  void setSetpoint(uint8_t setpoint);

  void encoderUpdate();
  void calculateSpeed();
};

void registerEncoderISRs();
void forward(uint8_t);
void backward(uint8_t);
void coast();
void activeBreak();
void calculateMotorSpeeds();

#endif // MOTOR_H
