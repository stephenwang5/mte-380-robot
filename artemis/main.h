#ifndef MAIN_H
#define MAIN_H

#include <Wire.h>
#include <MPU9250.h>
#include "control.h"
#include "motor.h"
#include "tof.h"
#include "timer.h"
#include "imu.h"

constexpr uint32_t I2C_FREQ = 400000;
constexpr uint16_t TICKS_PER_REV = 2100;

constexpr int robotWidth = 4;
constexpr int wheelRadius = 3;

extern Motor leftMotor;
extern Motor rightMotor;
extern TwoWire i2c;
extern ToF tof;
extern MPU9250 imu;

#endif // MAIN_H
