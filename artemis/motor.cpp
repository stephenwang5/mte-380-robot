#include "motor.h"

// typedef delay f_delay;
#define f_delay delay

namespace Motor {

  void begin() {
    attachInterrupt(Left::enc, Left::isr, LOW);
    attachInterrupt(Right::enc, Right::isr, LOW);
  }
  enum Direction {
    FORWARD = 1,
    BACKWARD = -1,
  };

  namespace Left {
    static int64_t encoder = 0;
    static int8_t direction = FORWARD;

    void isr() {
      encoder += direction;
    }

    void forward(uint8_t pwm) {
      analogWrite(f, pwm);
      analogWrite(b, 0);
      direction = FORWARD;
      f_delay(100);
    }
    void stop() {
      analogWrite(f, 0);
      analogWrite(b, 0);
      f_delay(100);
    }
    void backward(uint8_t pwm) {
      analogWrite(f, 0);
      analogWrite(b, pwm);
      direction = BACKWARD;
      f_delay(100);
    }
  }

  namespace Right {
    static int64_t encoder = 0;
    static int8_t direction = FORWARD;

    void isr() {
      encoder += direction;
    }

    void forward(uint8_t pwm) {
      analogWrite(f, pwm);
      analogWrite(b, 0);
      direction = FORWARD;
      f_delay(100);
    }
    void stop() {
      analogWrite(f, 0);
      analogWrite(b, 0);
      f_delay(100);
    }
    void backward(uint8_t pwm) {
      analogWrite(f, 0);
      analogWrite(b, pwm);
      direction = BACKWARD;
      f_delay(100);
    }
  }
}
