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
void calculateMotorSpeeds();
rtos::Thread motorControlTask;
void controlMotor();
rtos::Timer tofInputTask;
rtos::Timer imuInputTask;
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
void courseAdjustment();
void motorDemo();

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
  motorControlTask.start(controlMotor);

  Serial.println("left,right");

}

void loop() {

  Serial.print(leftMotor.encoder);
  Serial.print(",");
  Serial.println(rightMotor.encoder);
  delay(100);

}

void calculateMotorSpeeds() {
  while (1) {
    leftMotor.calculateSpeed();
    rightMotor.calculateSpeed();
    rtos::ThisThread::sleep_for(1ms);
  }
}

void controlMotor() {
  static uint8_t pwm = 0;
  while (1) {
    if (!pwm) {
      leftMotor.encoder = 0;
      rightMotor.encoder = 0;
    }
    leftMotor.rotateCCW(pwm >> 2);
    rightMotor.rotateCW(pwm++ >> 2);
    rtos::ThisThread::sleep_for(10ms);
  }
  
}

void motorDemo() {
  turnInPlace(-90);
  delay(2000);
  forward(100);
  delay(2000);
  coast();
  delay(2000);
  turnInPlace(90);
  delay(2000);
  backward(100);
  delay(2000);
  coast();
  delay(4000);
}
