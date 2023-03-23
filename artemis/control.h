#ifndef CONTROL_H
#define CONTROL_H

#include <PID_v1.h>

// Variables for general PID used to match encoder ticks from both motors
extern double pid_setpoint, pid_output, pid_input;
extern double Kp, Ki, Kd;
extern PID straightLinePID;
extern uint8_t target_pwm; //0-255

extern uint8_t leftpwm, rightpwm;

void spinCCW(uint8_t pwm);
void spinCW(uint8_t pwm);

#endif // CONTROL_H
