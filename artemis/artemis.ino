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
float homeMagZLocation = 0;

// define states and initialize the state variable

rtos::Thread bleTask;
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
  registerEncoderISRs();

  throwbotState = IDLE;

  motorSpeedTask.start(calculateMotorSpeeds);
  tofInputTask.start(readToF);
  imuInputTask.start(imuReadLoop);
  debugPrinter.start(printDebugMsgs);
  bleTask.start(BLEComm);
  //launchDetection.start(DetectLaunch);

  straightDrivePID.SetOutputLimits(-255 + pwm_straight_drive, 255 - pwm_straight_drive);
  straightDrivePID.SetMode(AUTOMATIC);

}

void loop() {
  // state transition logic here
  // threads are statically allocated then started/stopped here
  if (throwbotState == IDLE) {

    // wait for initial measurements to come through
    sleep_for(200ms);

    findOrientation();

    // assuming that there is enough time for the buffer to fill up
    // and no 0s will be used as the home position
    float magZBuf[10] = {0};
    uint8_t bufIdx = 0;

    while (imuMagnitude() > freeFallThreshold) {

      magZBuf[bufIdx] = findHeading();
      // use the measurement in the past 1 second as the home orientation
      homeMagZLocation = magZBuf[(bufIdx+9) % 10];

      bufIdx++;

      sleep_for(100ms);
    }
    throwbotState = READY;
  } else if (throwbotState == READY) {

    // in the air
    sleep_for(3s);

    // optional: spin to correct
    spinCW(40, 40);
    sleep_for(3s);
    throwbotState = IDLE;
    // findOrientation();
    // turnInPlaceByMag((homeHeading + 3.14) % 1.57, 38);
    // // TODO: spin back to home position
    // findOrientation();
    // throwbotState = SURVEY;

  } else if (throwbotState == SURVEY) {
    int num_turns = 0, degrees = 30;
    while(!tofMatch || num_turns < 2*360/degrees) {
      surveyTurnTask.start(controlMotorSpeedsForTurning); // will just turn robot slowly without stopping
      //TurnInPlaceByNumDegrees(degrees); // resulting in ~ 45 degrees of rotation in real life, therefore keep the number of degrees at 30 or less.
      //sleep_for(300ms);
      //num_turns++;
    }
    surveyTurnTask.terminate();
    if (tofMatch) {
      throwbotState = CONFIRM;
    } else if (num_turns > 2*360/degrees) { // robot has not been able to locate the pole for the past two full rotation. The robot needs to move
      throwbotState = MOVE_TO_NEW_LOCATION;
    }
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
    driveStraightTask.start(driveStraight);

    tofDataLock.lock();
    if (tofData.distance_mm[2,4] < 100)
    {
       throwbotState = STOP;
    }
    tofDataLock.unlock();
   
  } else if (throwbotState == STOP) {
    driveStraightTask.terminate();
    coast();
  }
 
  delay(50);
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
    // Serial.print(",");
    // Serial.println();

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
    // Serial.print("left encoder count ");
    // Serial.print(leftMotor.encoder);
    // Serial.print(", output pwm ");
    // Serial.print(leftpwm);
    // Serial.print(" , speed ");
    // Serial.println(leftMotor.speed);
    // Serial.print("rightMotor encoder count ");
    // Serial.print(rightMotor.encoder);
    // Serial.print(", output pwm ");
    // Serial.print(rightpwm);
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

    // Serial.print("ax: ");
    // Serial.print(imu.ax);
    // Serial.print(" ay: ");
   // Serial.print(imu.ay);
    // Serial.print("az: ");
    // Serial.println(imu.az);

    // Serial.println(orientation);

    if (orientation == IMU_FACE_UP)
    {
      Serial.println("Face UP");
    } else if (orientation == IMU_FACE_DOWN) {
      Serial.println("Face DOWN");
    }
  
    rtos::ThisThread::sleep_for(250ms);
  }
}

