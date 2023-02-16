#ifndef MOTOR_H
#define MOTOR_H
#include "encoder.h"

class Motor{
  public:
    Motor(int a, int b, Encoder enc): PinA(a), PinB(b), encoder(enc) {
      
      // pidController = PID(&(encoder.encoderSpeed), &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
      // pidController.SetMode(AUTOMATIC);
      // pidController.SetSampleTime(100);
    }

    Encoder encoder;
    const int PinA;
    const int PinB;
    int direction;
    //PID pidController;

    double Setpoint, Input, Output;
    double lastEncoderCount = 0;
    double Kp=1, Ki=0.1, Kd=0.01;

    void begin();
    void clockwise(uint8_t);
    void coast();
    void counterclockwise(uint8_t);
    //void controlSpeed();

    void setSetpoint(uint8_t setpoint);
};

#endif // MOTOR_H
