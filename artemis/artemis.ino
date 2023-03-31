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
  initBLE();

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
  bleTask.start(BLEComm);

  straightDrivePID.SetOutputLimits(-255 + pwm_straight_drive, 255 - pwm_straight_drive);
  straightDrivePID.SetMode(AUTOMATIC);

  // wait for initial measurements to come through
  sleep_for(200ms);
  Serial.println("all systems go");

}

int16_t minDistance(uint8_t row) {
  tofDataLock.lock();
  int16_t distance = tofData.distance_mm[row*8 + 0];
  for (uint8_t i = 1; i < 8; i++) {
    distance = min(distance, tofData.distance_mm[row*8 + i]);
  }
  tofDataLock.unlock();
  return distance;
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

      sleep_for(100ms);
    } while (imuMagnitudeNumber > freeFallThreshold);

    throwbotState = READY;

  } else if (throwbotState == TEST) {

    // float angle[] = {0, 3.14/2, 3.14, 3*3.14/2-0.1, -3.14/2};
    float angle[] = {
      0,
      90,
      180,
      260,
      -45,
    };
    int idx = 0;
    while (1) {
      homeHeading = angle[idx];
      turnInPlaceByMag(angle[idx], 22);
      idx = (idx+1) % 5;
      sleep_for(3s);
    }
    // sleep_for(1s);

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

    // optional: spin to correct
    // spinCW(40, 40);
    // sleep_for(1s);
    // Serial.println("after sleeping");
    // coast();
    // throwbotState = IDLE;
    // Serial.println("exiting ready");
    // if (orientation == IMU_FACE_UP) {
    //   turnInPlaceByMag(homeHeading, 38);
    // } else {
    //   turnInPlaceByMag(-homeHeading + 3.14, 38);
    // }
    // // TODO: spin back to home position
    throwbotState = SURVEY;

  } else if (throwbotState == SURVEY) {
    surveyCtr++;
    // int num_turns = 0, degrees = 30;
    uint32_t time = millis();
    // rightMotor.rotateCW(20);
    spinCCW(20, 0);
    // while(tofMatch < 0 && num_turns < 2*360/degrees) {
    bool timeout = false;
    bool match = false;
    do {
      timeout = millis() - time > 10000;
      match = tofMatch > -1;
      sleep_for(100ms);
    } while (!match && !timeout && surveyCtr < 3);
      // surveyTurnTask.start(controlMotorSpeedsForTurning); // will just turn robot slowly without stopping
      //TurnInPlaceByNumDegrees(degrees); // resulting in ~ 45 degrees of rotation in real life, therefore keep the number of degrees at 30 or less.
      //num_turns++;
    coast();
    throwbotState = (match) ? CONFIRM : WANDER;
    if (surveyCtr > 2) {
      throwbotState = WANDER;
    } else if (match) {
      throwbotState = CONFIRM;
    } else {
      throwbotState = WANDER;
    }
    // surveyTurnTask.terminate();
    // if (tofMatch) {
    //   throwbotState = CONFIRM;
    // } else if (num_turns > 2*360/degrees) { // robot has not been able to locate the pole for the past two full rotation. The robot needs to move
    //   throwbotState = MOVE_TO_NEW_LOCATION;
    // }
  } else if (throwbotState == CONFIRM) {
    // double back because the robot overshoots the target
    // spinCW(25);
    coast();
    uint8_t ctr = 0;
    for (uint8_t i = 0; i < 3; i++) {
      if (tofMatch > -1) {
        ctr++;
      }
      sleep_for(150ms);
    }
    throwbotState = (ctr>2) ? DRIVE : SURVEY;

  } else if (throwbotState == DRIVE) {
    osStatus status = driveStraightTask.start(driveToPole);
    Serial.println(status);


    int16_t distance;
    do {
      tofDataLock.lock();
      distance = tofData.distance_mm[2*8 + 0];
      for (uint8_t i = 1; i < 6; i++) {
        distance = min(distance, tofData.distance_mm[3*8 + i]);
      }
      tofDataLock.unlock();
      sleep_for(50ms);
    } while (distance > 50);

    driveStraightTask.terminate();
    driveStraightTask.join();
    coast();
    throwbotState = STOP;
   
  } else if (throwbotState == WANDER) {

    surveyCtr = 0;

    spinCW(15, 15);

    int16_t distance;
    uint8_t bottomRow = orientation==IMU_FACE_UP ? 0 : 7;
    uint32_t time = millis();
    uint16_t ctr = 0;

    do {
      distance = minDistance(bottomRow);
      ctr++;
      sleep_for(100ms);


    } while (distance < 800 && millis()-time < 10000);

    if (ctr > 1)
      sleep_for(1s);

    forward(50, 50);
    time = millis();
    do {
      distance = minDistance(bottomRow);
      sleep_for(100ms);
    } while (distance > 300 && millis() - time < 2000);
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

    Serial.print(throwbotState);
    Serial.print(",");
    // Serial.print(homeHeading);
    // Serial.print(",");
    // Serial.print(avgHeading);
    // Serial.print(findHeading());
    // printBuf<float>(magHeadingBuf, 10);
    // Serial.print(" magnitude: ");
    // Serial.print(imuMagnitudeNumber);

    // tofDataLock.lock();
    printBuf<int16_t>(tofData.distance_mm, 8, 8);
    Serial.println();
    printBuf<uint16_t>(tofData.range_sigma_mm, 8, 8);
    // Serial.println();
    // printBuf<uint8_t>(tofData.reflectance, 8, 8);
    // Serial.println();
    // tofDataLock.unlock();
    // printBuf<float>(tofDotProduct, 4);
    // Serial.println(tofMatch);
  
    Serial.println();
    rtos::ThisThread::sleep_for(300ms);
  }
}

