#include "main.h"

using namespace rtos::ThisThread;

//bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
// Motor(pinA, pinB, pinEncoder);
Motor leftMotor(D35, D6, D14);
Motor rightMotor(D29, D11, D23);

// TwoWire(pinSDA, pinSCL)
TwoWire i2c(D25, D27);
rtos::Mutex i2cLock("I2C Lock");

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
rtos::Mutex tofDataLock("tof data");
MPU9250 imu(MPU9250_ADDRESS_AD0, i2c, I2C_FREQ);

enum ThrowBotState {
  IDLE,
  READY,
  SURVEY,
  CONFIRM,
  DRIVE,
} throwbotState; // define states and initialize the state variable

rtos::Thread bleTask;
rtos::Thread motorSpeedTask;
rtos::Thread motorControlTask;
rtos::Thread tofInputTask;
rtos::Thread imuInputTask(osPriorityNormal, OS_STACK_SIZE, nullptr, "imu");
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
rtos::Thread debugPrinter;
void printDebugMsgs();

template<typename T>
void printBuf(const T* const buf, uint8_t col, uint8_t row=1) {
  for (int j = 0; j < row; j++) {
    for (int i = 0; i < col; i++) {
      Serial.print(buf[i + j*col]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

template<typename T>
void printBufBytes(const T* const buf, uint8_t len) {
  uint16_t size = len * sizeof(T);
  Serial.write((uint8_t*)buf, size);
  Serial.write("\n");
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
  initBLE();

  leftMotor.begin();
  rightMotor.begin();
  registerEncoderISRs();

  throwbotState = IDLE;

  findOrientation();

  motorSpeedTask.start(calculateMotorSpeeds);
  tofInputTask.start(readToF);
  imuInputTask.start(imuReadLoop);
  bleTask.start(BLEComm);
  // debugPrinter.start(printTof);

}

void loop() {

  if (throwbotState == IDLE) {

    while (imuMagnitude() > freeFallThreshold) {
      sleep_for(100ms);
    }
    throwbotState = READY;

  } else if (throwbotState == READY) {

    sleep_for(3s);
    findOrientation();
    throwbotState = SURVEY;

  } else if (throwbotState == SURVEY) {

    spinCCW(25);
    while (tofMatch < 0) {
      sleep_for(30ms);
    }
    throwbotState = CONFIRM;

  } else if (throwbotState == CONFIRM) {

    // double back because the robot overshoots the target
    spinCW(25);
    sleep_for(500ms);
    coast();
    uint8_t ctr = 0;
    while (ctr >= 0 && ctr < 3) {
      if (tofMatch < 0) {
        ctr--;
      } else {
        ctr++;
      }
      sleep_for(150ms);
    }
    throwbotState = (ctr==3) ? DRIVE : SURVEY;

  } else if (throwbotState == DRIVE) {

    forward(40);
    sleep_for(500ms);
    throwbotState = SURVEY;

  }

}

void printDebugMsgs() {
  while (1) {
    // Serial.print(leftMotor.speed);
    // Serial.print(",");
    // Serial.print(rightMotor.speed);
    // Serial.println();

    Serial.print(imu.yaw);
    Serial.print(",");
    Serial.print(imu.pitch);
    // Serial.print(",");
    // Serial.print(imu.roll);
    Serial.println();

    rtos::ThisThread::sleep_for(50ms);
  }
}

void printTof() {
  while (1) {
    rtos::ThisThread::sleep_for(100ms);

    tofDataLock.lock();

    // if (tofMatch >= 0) {
    //   Serial.println(tofMatch);
    //   // printBuf<float>(tofDotProduct, 4);
    //   printBuf<float>(tofNormalized, 8, 8);
    //   Serial.println();
    // }
    // printBuf<float>(tofDotProduct, strideLen);
    // Serial.print(bufMax(tofDotProduct, strideLen));
    // Serial.println();
    // printBuf<float>(tofNormalized, 8, 8);
    // Serial.println();
    // printBuf<float>(tofNormalized, 64);
    // printBufBytes<int16_t>(tofData.distance_mm, 64);
    printBuf<uint16_t>(tofData.range_sigma_mm, 8, 8);
    Serial.println();

    tofDataLock.unlock();
  }
}
