#ifndef MOTOR_H
#define MOTOR_H

namespace Motor {
  void begin();

  namespace Left {
    const PinName enc = D24;
    const PinName f = D35;
    const PinName b = D34;

    void forward(uint8_t);
    void stop();
    void backward(uint8_t);
    void isr();
  }

  namespace Right {
    const PinName enc = D5;
    const PinName f = D29;
    const PinName b = D11;

    void forward(uint8_t);
    void stop();
    void backward(uint8_t);
    void isr();
  }
}

#endif // MOTOR_H
