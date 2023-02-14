#ifndef ENCODER_H
#define ENCODER_H

enum Direction {
  CW = 1,
  CCW = -1,
};

class Encoder {
  public:
    Encoder(int A): pinA(A) {
      pinMode(pinA, INPUT);
    }
    long encoderPos = 0;
    const int pinA;
    double speed = 0;
    //const int pinB;// not going to use pinB since it is broken on the right motor

    void encoderUpdate(int motorDir);
};

#endif // ENCODER_H
