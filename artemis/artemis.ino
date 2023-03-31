#include "main.h"

using namespace rtos::ThisThread;

//bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
// Motor(pinA, pinB, pinEncoder);
Motor leftMotor(D35, D6, D5);
Motor rightMotor(D29, D11, D24);

// TwoWire(pinSDA, pinSCL)
TwoWire i2c(D25, D27);
rtos::Mutex i2cLock("I2C Lock");

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
rtos::Mutex tofDataLock("tof data");
MPU9250 imu(MPU9250_ADDRESS_AD0, i2c, I2C_FREQ);
rtos::Mutex imuLock("imu data");
float homeHeading = 0;
float homeMagX = 0;
float homeMagY = 0;
float imuMagnitudeNumber = 0;
float magHeadingBuf[10] = {0}; // has to allocate this buffer outside if scope
uint8_t surveyCtr = 0;

// define states and initialize the state variable

rtos::Thread bleTask;
rtos::Thread motorEncTask;
rtos::Thread motorSpeedTask;
rtos::Thread surveyTurnTask;
rtos::Thread driveStraightTask;
rtos::Thread tofInputTask;
rtos::Thread imuInputTask(osPriorityNormal, OS_STACK_SIZE, nullptr, "imu");
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
rtos::Thread debugPrinter;
rtos::Thread launchDetection;
void printDebugMsgs();

ThrowbotState throwbotState;

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
  // initBLE();

  leftMotor.begin();
  rightMotor.begin();
  coast();
  registerEncoderISRs();

  throwbotState = IDLE;

  // motorEncTask.start(updateBothEncoders); // polling update
  // motorSpeedTask.start(calculateMotorSpeeds);
  tofInputTask.start(readToF);
  imuInputTask.start(imuReadLoop);
  debugPrinter.start(printDebugMsgs);
  // bleTask.start(BLEComm);

  straightDrivePID.SetOutputLimits(-255 + pwm_straight_drive, 255 - pwm_straight_drive);
  straightDrivePID.SetMode(AUTOMATIC);

  // wait for initial measurements to come through
  sleep_for(200ms);
  // Serial.println("all systems go");

}

void loop() {
  // state transition logic here
  // threads are statically allocated then started/stopped here


  if (throwbotState == IDLE) {

    surveyCtr = 0;

    // assuming that there is enough time for the buffer to fill up
    // and no 0s will be used as the home position
    uint8_t bufIdx = 0;

    do {
      imuMagnitudeNumber = imuMagnitude();

      // magHeadingBuf[bufIdx] = findHeading();
      // // use the measurement in the past 1 second as the home orientation
      // homeHeading = magHeadingBuf[(bufIdx+9) % 10];

      // bufIdx = (bufIdx+1) % 10;

      sleep_for(10ms);
    } while (imuMagnitudeNumber > freeFallThreshold);

    throwbotState = READY;

  } else if (throwbotState == READY) {

    // in the air
    sleep_for(2s);

    while (abs(imu.az) < 0.5) {
      leftMotor.rotateCW(200);
      rightMotor.rotateCCW(200);
      sleep_for(500ms);
      coast();
      sleep_for(1s);
    }

    findOrientation();
    while (orientation == UNKNOWN) {
      findOrientation();
      sleep_for(100ms);
    }

    throwbotState = SURVEY;

  } else if (throwbotState == SURVEY) {
    surveyCtr++;

    uint32_t time = millis();
    uint8_t ctr = 0;

    sleep_for(400ms); // first measurement
    while (ctr < 15 && tofMatch < 0){
      spinCCW(30,30);
      sleep_for(180ms);
      coast();

      sleep_for(400ms);

      ctr++;
    }

    coast();

    if (surveyCtr > 2) {
      throwbotState = WANDER;
    } else if (tofMatch > -1) {
      throwbotState = CONFIRM;
    } else {
      throwbotState = WANDER;
    }

  } else if (throwbotState == CONFIRM) {
    // double back because the robot overshoots the target
    // spinCW(25);
    coast();
    uint8_t ctr = 0;
    for (uint8_t i = 0; i < 3; i++) {
      if (tofMatch > -1) {
        ctr++;
      }
      sleep_for(200ms);
    }
    throwbotState = (ctr>2) ? DRIVE : SURVEY;

  } else if (throwbotState == DRIVE) {

    forward(pwm_straight_drive, pwm_straight_drive);

    int16_t distance;
    do {
      distance = minDistance(3);

      tofDataLock.lock();
      int pid_output = (1.5 - tofMatch) * 10;
      if (tofMatch == 3) { // too far right, speed up left wheel
        forward(pwm_straight_drive + abs(pid_output), pwm_straight_drive);
      } else if (tofMatch == 0) { // too far left, speed up right wheel
        forward(pwm_straight_drive, pwm_straight_drive + abs(pid_output));
      } else {
        forward(pwm_straight_drive, pwm_straight_drive);
      }
      tofDataLock.unlock();

      sleep_for(50ms);
    } while (distance > 50);

    coast();
    throwbotState = STOP;
   
  } else if (throwbotState == WANDER) {

    surveyCtr = 0;

    spinCW(20, 20);

    int16_t distance;
    uint8_t bottomRow = orientation==IMU_FACE_UP ? 0 : 7;
    uint32_t time = millis();
    uint16_t ctr = 0;

    do {
      distance = avgDistance(bottomRow);
      ctr++;
      sleep_for(100ms);

    } while (distance < 900 && millis()-time < 10000);

    if (ctr > 1)
      sleep_for(500ms);

    forward(50, 50);
    time = millis();
    do {
      distance = avgDistance(bottomRow);
      sleep_for(100ms);
    } while (distance > 700 && millis() - time < 2000);
    coast();

    throwbotState = SURVEY;
   
  } else if (throwbotState == STOP) {
    coast();
    throwbotState = IDLE;
    // while (1);
  }
 
  sleep_for(50ms);
}

void printDebugMsgs() {
  while (1) {

    // Serial.print(imu.ax);
    // Serial.print(",");
    // Serial.print(imu.ay);
    // Serial.print(",");
    // Serial.print(imu.az);
    // Serial.print(",");
    // Serial.print(imu.gx);
    // Serial.print(",");
    // Serial.print(imu.gy);
    // Serial.print(",");
    // Serial.print(imu.gz);
    // Serial.print(",");
    // Serial.print(imu.mx);
    // Serial.print(",");
    // Serial.print(imu.my);
    // Serial.print(",");
    // Serial.print(imu.mz);
    // Serial.println();

    // Serial.print(imu.yaw);
    // Serial.print(",");
    // Serial.print(imu.roll);
    // Serial.print(",");
    // Serial.print(imu.pitch);

    // Serial.print("tof 1: ");
    // Serial.println(tofData.distance_mm[1]);
    // Serial.print("tof 2: ");
    // Serial.println(tofData.distance_mm[2]);
    // Serial.print("tof 3: ");
    // Serial.println(tofData.distance_mm[3]);

    // Serial.print(leftMotor.encoder);
    // Serial.print(",");
    // Serial.print(rightMotor.encoder);
    // Serial.println();

    // printBuf<int16_t>(tofData.distance_mm, 8, 8);
    // Serial.println("\n\n\n");

    // Serial.print("leftMotor input speed ");
    // Serial.println(leftMotor.speed);
    // Serial.print("leftMotor output pwm ");
    // Serial.println(leftMotor.Output);
    // Serial.print("rightMotor input speed ");
    // Serial.println(rightMotor.speed);
    // Serial.print("rightMotor output pwm ");
    // Serial.println(rightMotor.Output);

    // Serial.print("PID output ");
    // Serial.println(pid_output);
    // Serial.print(", input ");
    // Serial.print(pid_input);
    // Serial.print("left encoder count ");
    // Serial.print(leftMotor.encoder);
    // Serial.print(", left ");
    // Serial.print(leftpwm);
    // Serial.print(" , speed ");
    // Serial.println(leftMotor.speed);
    // Serial.print("rightMotor encoder count ");
    // Serial.print(rightMotor.encoder);
    // Serial.print(", right ");
    // Serial.print(rightpwm);
    // Serial.println();
    // Serial.print(", speed ");
    // Serial.println(rightMotor.speed);

    // Serial.print("P: ");
    // Serial.print(P);
    // Serial.print(", I: ");
    // Serial.print(I);
    // Serial.print(", D: ");
    // Serial.println(D);
    // Serial.print("PID output ");
    // Serial.println(pid_output);

    // Serial.println(orientation);

    // if (orientation == IMU_FACE_UP)
    // {
    //   Serial.println("Face UP");
    // } else if (orientation == IMU_FACE_DOWN) {
    //   Serial.println("Face DOWN");
    // }

    // Serial.print(throwbotState);
    // Serial.print(",");
    // Serial.print(homeHeading);
    // Serial.print(",");
    // Serial.print(avgHeading);
    // Serial.print(findHeading());
    // printBuf<float>(magHeadingBuf, 10);
    // Serial.print(" magnitude: ");
    // Serial.print(imuMagnitudeNumber);

    tofDataLock.lock();
    // printBufBytes<int16_t>(tofData.distance_mm, 64);
    printBuf<int16_t>(tofData.distance_mm, 8, 8);
    Serial.println();
    printBuf<uint16_t>(tofData.range_sigma_mm, 8, 8);
    Serial.println();
    // printBuf<uint8_t>(tofData.reflectance, 8, 8);
    // Serial.println();
    printBuf<float>(tofDotProduct, 4);
    tofDataLock.unlock();
    // Serial.println(tofMatch);
  
    Serial.println();
    rtos::ThisThread::sleep_for(500ms);
  }
}

