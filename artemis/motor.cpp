#include "ThisThread.h"
#include "motor.h"

#include "main.h"

inline bool closeTo(float a, float b) {
  return abs(a-b) < 3e-1;
}

int target_turn_pwm = 19;
// Variables for general PID used to match encoder ticks from both motors
double pid_setpoint = 0; double pid_output, pid_input;
PID straightDrivePID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
uint8_t leftpwm, rightpwm;

Motor::Motor(PinName a, PinName b, PinName enc): pinA(a), pinB(b), encoderPin(enc), pidController(&speed, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {}

void Motor::begin() {
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(encoderPin, INPUT);
  pidController.SetOutputLimits(0, 255-target_turn_pwm);
  pidController.SetMode(AUTOMATIC);
}

void Motor::rotateCW(uint8_t pwm) {
  analogWrite(pinA, pwm);
  analogWrite(pinB, 0);
  direction = CW;
}

void Motor::rotateCCW(uint8_t pwm) {
  analogWrite(pinA, 0);
  analogWrite(pinB, pwm);
}

void Motor::coast() {
  analogWrite(pinA, 0);
  analogWrite(pinB, 0);
}

void Motor::activeBreak() {
  analogWrite(pinA, 255);
  analogWrite(pinB, 255);
}

void Motor::encoderUpdate() {
  // CW increments and CCW decrements
  bool val = digitalRead(encoderPin);
  encoder += direction * (val ^ encMem);
  encMem = val;
}

void Motor::calculateSpeed() { // uses rolling average filter

  int32_t curTime = micros();
  encBuf[encBufIdx] = encoder;
  encBufTime[encBufIdx] = curTime;
  encBufIdx = (encBufIdx + 1) % encBufLength;

  if (!encBufInitialized) {
    if (encBufIdx == (encBufLength-1)) {
      encBufInitialized = true;
      encBuf[encBufIdx] = encoder;
      encBufTime[encBufIdx] = curTime + 1;
      encBufIdx = (encBufIdx + 1) % encBufLength;
    } else {
      return;
    }
  }

  float sum = 0;
  for (int i = 0; i < encBufLength-1; i++) {
    int a = (i+encBufIdx) % encBufLength;
    int b = (a+1) % encBufLength;
    sum += 1e6f * (encBuf[b] - encBuf[a]) / (encBufTime[b] - encBufTime[a]);
  }

  speed = abs(sum / (encBufLength-1) / TICKS_PER_REV); // rps

}

void updateBothEncoders() {
  leftMotor.encoderUpdate();
  rightMotor.encoderUpdate();
  rtos::ThisThread::sleep_for(1ms);
}

static void updateRightEncoder() {
  rightMotor.encoderUpdate();
}

static void updateLeftEncoder() {
  leftMotor.encoderUpdate();
}

void registerEncoderISRs() {
  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPin), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPin), updateRightEncoder, RISING);
}

void forward(uint8_t pwmL, uint8_t pwmR){
  if (orientation == IMU_FACE_DOWN) {
    leftMotor.rotateCCW(pwmR * leftFactor);
    rightMotor.rotateCW(pwmL * rightFactor);
  } else {
    leftMotor.rotateCW(pwmL * leftFactor);
    rightMotor.rotateCCW(pwmR * rightFactor);
  }
}

void coast(){
  leftMotor.coast();
  rightMotor.coast();
}

void activeBreak() {
  leftMotor.activeBreak();
  rightMotor.activeBreak();
}

void calculateMotorSpeeds() {
  while (1) {
    leftMotor.calculateSpeed();
    rightMotor.calculateSpeed();
    rtos::ThisThread::sleep_for(2ms);
  }
}

double Kp=0, Ki=0, Kd=0;

void driveToPole() {
  pid_setpoint = 1.5;
  Kp=6; Ki=0; Kd=0;

  forward(pwm_straight_drive, pwm_straight_drive);
  while(1){
    int pid_output = (1.5 - tofMatch) * Kp;
    // pid_input = tofMatch; // 0,1,2 or 3
    // 0 means pole is right of middle
    // 1 or 2 mean that the pole is relatively in the middle
    // 3 means that pole is left of middle

    // straightDrivePID.Compute();

    if (tofMatch == 3) { // too far right, speed up left wheel
      leftpwm = pwm_straight_drive + abs(pid_output);
      rightpwm = pwm_straight_drive;
      forward(pwm_straight_drive + abs(pid_output), pwm_straight_drive);
    } else if (tofMatch == 0) { // too far left, speed up right wheel
      leftpwm = pwm_straight_drive;
      rightpwm = pwm_straight_drive + abs(pid_output);
      forward(pwm_straight_drive, pwm_straight_drive + abs(pid_output));
    } else {
      leftpwm = pwm_straight_drive;
      rightpwm = pwm_straight_drive;
      forward(pwm_straight_drive, pwm_straight_drive);
    }

    rtos::ThisThread::sleep_for(10ms);  
  }
}

void controlMotorSpeedsForTurning() {
  while(1){
    leftMotor.Setpoint= 0.2;
    rightMotor.Setpoint = 0.2;
      
    //Run the PID calculation
    leftMotor.pidController.Compute();
    rightMotor.pidController.Compute();

    //spinCCW(uint8_t pwmL, uint8_t pwmR)
    spinCCW(target_turn_pwm+leftMotor.Output, target_turn_pwm+rightMotor.Output);
    
    rtos::ThisThread::sleep_for(2ms);
  }  
}

// Function not finished
// void controlMotorSpeedsWithIMU() {
//   while(1) {
//     if (throwbotState == DRIVE) {

//       pid_input = imu.yaw;
//       pid_setpoint = drive_direction; // move this into main loop if we end up using this one.

//       //Run the PID calculation
//       pidController.Compute();
//       uint8_t left_pwm, right_pwm;
//       if (pid_output > 0) { // figure out which direction will give a positive error
//         left_pwm = target_pwm;
//         right_pwm = target_pwm - pid_output;
        
//         forward(target_pwm, target_pwm - pid_output);      
//       } else {
//         left_pwm = target_pwm;
//         right_pwm = 
//         forward(target_pwm - pid_output, target_pwm);
//       }
//     }
//     rtos::ThisThread::sleep_for(2ms);
//   }
// }

void driveStraight() {

  uint8_t target_pwm = 35; 
  pid_setpoint = 0;
  leftMotor.encoder = 0;
  rightMotor.encoder = 0;
  while(1) {
    //pid_setpoint is 0, defined above
    pid_input = abs(leftMotor.encoder) - rightMotor.encoder;
    // acc_error += pid_input;
    
    // P = pid_input * Kp;
    // I = acc_error * Ki * 0.001;
    // D = Kd * (pid_input - prev_error) / 0.001;

    //Run the PID calculation
    straightDrivePID.Compute();

    if (pid_input > 0){
      // leftpwm = target_pwm; 
      // rightpwm = target_pwm + abs(pid_output);
      forward(target_pwm, target_pwm + abs(pid_output));
    } else {
      forward(target_pwm + abs(pid_output), target_pwm);
      // leftpwm = target_pwm + abs(pid_output);
      // rightpwm = target_pwm;
    }
    // prev_error = pid_input; 
    rtos::ThisThread::sleep_for(2ms);
  }
}

constexpr int headingBufLen = 6;
float headingBuf[headingBufLen] = {0};
float avgHeading = 0;
uint8_t headingBufIdx = 0;

void filterHeading() {
  float heading = findHeading();
  uint8_t nextIdx = (headingBufIdx + 1) % headingBufLen;
  avgHeading += heading / headingBufLen;
  avgHeading -= headingBuf[nextIdx] / headingBufLen;

  headingBuf[headingBufIdx] = heading;
  headingBufIdx = nextIdx;
}

void turnInPlaceByMag(float targetMag, uint8_t pwm) {

  constexpr float tolerance = 15; // degrees

  float heading = findHeading();
  headingBufIdx = 0;
  avgHeading = heading;
  for (uint8_t i = 0; i < headingBufLen; i++) {
    headingBuf[i] = heading;
  }

  while (1) {
    rightMotor.rotateCCW(pwm);
    filterHeading();

    if (abs(avgHeading - targetMag) < tolerance) {

      activeBreak();

      int ctr = 0;
      for (int i = 0; i < 5; i++) {
        filterHeading();

        if (abs(avgHeading - targetMag) < tolerance)
          ctr++;

        rtos::ThisThread::sleep_for(10ms);
      }

      if (ctr > 3)
        break;
    }
    rtos::ThisThread::sleep_for(10ms);
  }

  coast();
}

void TurnInPlaceByNumDegrees(float degrees){
  uint8_t target_pwm = 35; 

  bool clockwise = degrees > 0;
  float turnDist = robotWidth*(abs(degrees))/360.0f; // turn distance of each wheel
  uint16_t numticks = uint16_t((turnDist*TICKS_PER_REV)/(wheelDiameter)); // number of encoder ticks needed to turn provided number of degrees

  const int R_startPos = rightMotor.encoder;
  const int L_startPos = leftMotor.encoder;

  if (clockwise) { // Left wheel goes forward, right wheel goes back => both wheels spin CCW
    rightMotor.rotateCCW(target_pwm); // right motor spins slower than left motor by this factor
    leftMotor.rotateCCW(target_pwm);

  } else { // Right wheel goes forward, left wheel goes back => both wheels spin CW
    rightMotor.rotateCW(target_pwm);
    leftMotor.rotateCW(target_pwm);
  }

  bool Rturning  = true, Lturning = true;
  while (Rturning || Lturning) {
    if (abs(rightMotor.encoder - R_startPos) >= numticks) {
      rightMotor.coast();
      Rturning = false;
    }
    if (abs(leftMotor.encoder - L_startPos) >= numticks) {
      leftMotor.coast();
      Lturning = false;
    }
  }
}

void spinCCW(uint8_t pwmL, uint8_t pwmR) {
  pwmL *= leftFactor;
  pwmR *= rightFactor;
  if (orientation == IMU_FACE_DOWN) {
    leftMotor.rotateCW(pwmL * leftFactor);
    rightMotor.rotateCW(pwmR * rightFactor);
  } else {
    leftMotor.rotateCCW(pwmL * leftFactor);
    rightMotor.rotateCCW(pwmR * rightFactor);
  }
}

void spinCW(uint8_t pwmL, uint8_t pwmR) {
  pwmL *= leftFactor;
  pwmR *= rightFactor;
  if (orientation == IMU_FACE_DOWN) {
    leftMotor.rotateCCW(pwmL * leftFactor);
    rightMotor.rotateCCW(pwmR * rightFactor);
  } else {
    leftMotor.rotateCW(pwmL * leftFactor);
    rightMotor.rotateCW(pwmR * rightFactor);
  }
}




