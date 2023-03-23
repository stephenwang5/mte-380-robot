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

ThrowbotState throwbotState; // define states and initialize the state variable

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

  throwbotState = SURVEY;

  findOrientation();

  imuInputTask.start(imuReadLoop);
  bleTask.start(BLEComm);
  debugPrinter.start(printTof);

}

void loop() {

  if (throwbotState == IDLE) {

    // wait for initial measurements to come through
    sleep_for(200ms);

    while (imuMagnitude() > freeFallThreshold) {
      sleep_for(100ms);
    }
    throwbotState = READY;

  } else if (throwbotState == READY) {

    forward(100, 100);
    sleep_for(3s);
    findOrientation();
    throwbotState = SURVEY;

  } else if (throwbotState == SURVEY) {

    Serial.println("started surveying");

    spinCCW(25);
    while (tofMatch < 0) {
      sleep_for(30ms);
    }
    throwbotState = CONFIRM;

  } else if (throwbotState == CONFIRM) {

    // double back because the robot overshoots the target
    // spinCW(25);
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

    forward(40, 40);
    sleep_for(500ms);
    throwbotState = SURVEY;

  }

}

void printDebugMsgs() {
  while (1) {

    Serial.print(imu.ax);
    Serial.print(",");
    Serial.print(imu.ay);
    Serial.print(",");
    Serial.print(imu.az);
    Serial.print(",");
    Serial.print(imu.gx);
    Serial.print(",");
    Serial.print(imu.gy);
    Serial.print(",");
    Serial.print(imu.gz);
    Serial.print(",");
    Serial.print(imu.mx);
    Serial.print(",");
    Serial.print(imu.my);
    Serial.print(",");
    Serial.print(imu.mz);
    Serial.print(",");
    Serial.println();

    rtos::ThisThread::sleep_for(50ms);
  }
}
