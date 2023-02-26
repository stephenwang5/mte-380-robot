#include "main.h"

using namespace std::chrono_literals;

//bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
Motor leftMotor = Motor(D35, D6, D14);
Motor rightMotor = Motor(D29, D11, D23);

TwoWire i2c(D25, D27);
rtos::Mutex i2cLock("I2C Lock");

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
MPU9250 imu(MPU9250_ADDRESS_AD0, i2c, I2C_FREQ);

enum ThrowBotState {
  IDLE,
  READY,
  SURVEY,
  DRIVE,
} throwbotState; // define states and initialize the state variable

rtos::Thread motorSpeedTask;
rtos::Thread motorControlTask;
rtos::Thread tofInputTask;
rtos::Thread imuInputTask(osPriorityNormal, OS_STACK_SIZE, nullptr, "imu");
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
rtos::Thread debugPrinter;
void printDebugMsgs();

template<typename T>
void printBuf(const T* const buf, uint8_t col, uint8_t row=0) {
  for (int j = 0; j < row; j++) {
    for (int i = 0; i < col; i++) {
      Serial.print(buf[i + j*col]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void setup() {

  Serial.begin(115200);
  while(!Serial);
  Serial.println("firmware start!");

  // set broken motor pin to high impedance to stop interference with D6
  pinMode(34, INPUT);

  i2c.begin();
  i2c.setClock(I2C_FREQ);

  initToF();
  initIMU();

  leftMotor.begin();
  rightMotor.begin();
  registerEncoderISRs();

  throwbotState = IDLE;

  motorSpeedTask.start(calculateMotorSpeeds);
  tofInputTask.start(readToF);
  imuInputTask.start(imuReadLoop);
  motorControlTask.start(rampUpBothMotors);
  debugPrinter.start(printDebugMsgs);

}

void loop() {

  delay(1000);

}

void printDebugMsgs() {
  while (1) {
    // Serial.print(leftMotor.speed);
    // Serial.print(",");
    // Serial.print(rightMotor.speed);
    // Serial.println();

    // printBuf<int16_t>(tofData.distance_mm, 8, 8);
    // Serial.println("\n\n\n");

    Serial.print(imu.yaw);
    Serial.print(",");
    Serial.print(imu.pitch);
    Serial.print(",");
    Serial.print(imu.roll);
    Serial.println();

    rtos::ThisThread::sleep_for(500ms);
  }
}
