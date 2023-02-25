#include "main.h"

using namespace std::chrono_literals;

//bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
Motor leftMotor = Motor(D35, D6, D14);
Motor rightMotor = Motor(D29, D11, D23);

TwoWire i2c(D25, D27);
// ToF tof;
// MPU9250 imu(MPU9250_ADDRESS_AD0, i2c, I2C_FREQ);

enum ThrowBotState {
  IDLE,
  READY,
  SURVEY,
  DRIVE,
} throwbotState; // define states and initialize the state variable

rtos::Thread motorSpeedTask;
rtos::Thread motorControlTask;
rtos::Thread tofInputTask;
rtos::Thread imuInputTask;
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
rtos::Thread debugPrinter;
void printDebugMsgs();

void setup() {

  Serial.begin(115200);
  while(!Serial);
  Serial.println("firmware start!");

  // set broken motor pin to high impedance to stop interference with D6
  pinMode(34, INPUT);

  i2c.begin();
  i2c.setClock(I2C_FREQ);

  // tof.begin();
  // initIMU();

  leftMotor.begin();
  rightMotor.begin();
  registerEncoderISRs();

  throwbotState = IDLE;

  motorSpeedTask.start(calculateMotorSpeeds);
  motorControlTask.start(rampUpBothMotors);
  debugPrinter.start(printDebugMsgs);

  Serial.println("left,right");

}

void loop() {

  delay(1000);

}

void printDebugMsgs() {
  while (1) {
    Serial.print(leftMotor.speed);
    Serial.print(",");
    Serial.print(rightMotor.speed);
    Serial.println();
    rtos::ThisThread::sleep_for(50ms);
  }
}
