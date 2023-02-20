#ifndef MOTOR_H
#define MOTOR_H

enum Direction {
  CW = 1,
  CCW = -1,
};

class Motor{
public:

  int64_t encoder;
  int64_t prevEncoder;
  int32_t prevTime;
  PinName encoderPin;
  float speed;

  const PinName pinA;
  const PinName pinB;
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

void forward(uint8_t);
void backward(uint8_t);
void coast();
void activeBreak();

#endif // MOTOR_H
