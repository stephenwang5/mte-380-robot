#include "main.h"
#include "PID_v1.h"

//bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
// Motor(pinA, pinB, pinEncoder);
Motor leftMotor(D35, D6, D14);
Motor rightMotor(D29, D11, D23);

// TwoWire(pinSDA, pinSCL)
TwoWire i2c(D25, D27);
rtos::Mutex i2cLock("I2C Lock");

SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
MPU9250 imu(MPU9250_ADDRESS_AD0, i2c, I2C_FREQ);

// rtos::Thread motorSpeedTask;
rtos::Thread motorControlTask;
rtos::Thread tofInputTask;
rtos::Thread imuInputTask(osPriorityNormal, OS_STACK_SIZE, nullptr, "imu");
// correct path drift due to wheel slip using odometry estimation
rtos::Thread pathPlanningTask;
rtos::Thread debugPrinter;
void printDebugMsgs();

ThrowBotState throwbotState = IDLE;
bool firstDriveScan;
double drive_direction;

// Variables for general PID used to match encoder ticks from both motors
double pid_setpoint, pid_output, pid_input;
double Kp=1, Ki=0.1, Kd=0;
PID pidController(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
uint8_t target_pwm = 50; //0-255
uint8_t leftpwm, rightpwm;

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
  firstDriveScan = true;

  findOrientation();

  // motorSpeedTask.start(calculateMotorSpeeds);
  tofInputTask.start(readToF);
  imuInputTask.start(imuReadLoop);
  //motorControlTask.start(controlMotorSpeeds);
  motorControlTask.start(controlMotorSpeedsWithEncoderCount);
  debugPrinter.start(printDebugMsgs);

  pidController.SetOutputLimits(-255 + target_pwm, 255 - target_pwm);
  pidController.SetMode(AUTOMATIC);

}

void loop() {

  // state transition logic here
  // threads are statically allocated then started/stopped here
  switch (throwbotState){
    case IDLE:
    //
      throwbotState = READY;
    case READY:
    //
      throwbotState = SURVEY;
    case SURVEY:
    //
      throwbotState = DRIVE;
    case DRIVE:
    //
      if (firstDriveScan) {
        //drive_direction = imu.yaw; // for if we decide to steer with imu
        firstDriveScan = false;
        pid_setpoint = 0;
      }
        
      //throwbotState = STOP; // is currently commented out to test motor control threads
    case STOP:
      //
    default:
      //
      //do nothing
    break;
  }
 
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

    // Serial.print("leftMotor input speed ");
    // Serial.println(leftMotor.speed);
    // Serial.print("leftMotor output pwm ");
    // Serial.println(leftMotor.Output);
    // Serial.print("rightMotor input speed ");
    // Serial.println(rightMotor.speed);
    // Serial.print("rightMotor output pwm ");
    // Serial.println(rightMotor.Output);

    Serial.print("PID output ");
    Serial.println(pid_output);
    Serial.print("left encoder count ");
    Serial.print(leftMotor.encoder);
    Serial.print(", output pwm ");
    Serial.print(leftpwm);
    Serial.print(" , speed ");
    Serial.println(leftMotor.speed);
    Serial.print("rightMotor encoder count ");
    Serial.print(rightMotor.encoder);
    Serial.print(", output pwm ");
    Serial.print(rightpwm);
    Serial.print(", speed ");
    Serial.println(rightMotor.speed);

    // Serial.print("P: ");
    // Serial.print(P);
    // Serial.print(", I: ");
    // Serial.print(I);
    // Serial.print(", D: ");
    // Serial.println(D);
    // Serial.print("PID output ");
    // Serial.println(pid_output);

    rtos::ThisThread::sleep_for(500ms);
  }
}