#ifndef MAIN_H
#define MAIN_H

#include <Wire.h>
#include <MPU9250.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "control.h"
#include "motor.h"
#include "test.h"
#include "tof.h"
#include "timer.h"
#include "imu.h"

// uncomment to print more verbose messages
// #define DEBUG

using namespace std::chrono_literals;

constexpr uint32_t I2C_FREQ = 400000;
constexpr uint16_t TICKS_PER_REV = 356;

constexpr float robotWidth = 4.5;
constexpr float wheelRadius = 3.25;

extern Motor leftMotor;
extern Motor rightMotor;
extern TwoWire i2c;
extern rtos::Mutex i2cLock;
extern SparkFun_VL53L5CX tof;
extern VL53L5CX_ResultsData tofData;
extern rtos::Mutex tofDataLock;
extern MPU9250 imu;

#endif // MAIN_H
