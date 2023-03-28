#include "motor.h"

#include "main.h"

inline float abs(float a) {
  return a>0 ? a : -a;
}

inline bool closeTo(float a, float b) {
  return abs(a-b) > 1e-1;
}

int target_turn_pwm = 19;
// General PID used to match encoder ticks from both motors to get it to drive straight
double pid_setpoint, pid_output, pid_input;
double Kp, Ki, Kd;
PID straightDrivePID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT);
uint8_t pwm_straight_drive = 35; //0-255
// uint8_t leftpwm, rightpwm;

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
  encoder += direction;
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

static void updateRightEncoder() {
  rightMotor.encoderUpdate();
}

static void updateLeftEncoder() {
  leftMotor.encoderUpdate();
}

void registerEncoderISRs() {
  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPin), updateRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPin), updateLeftEncoder, RISING);
}

void forward(uint8_t pwmL, uint8_t pwmR){
  if (orientation == IMU_FACE_UP) { /// these might need to be swapped.
    leftMotor.rotateCCW(pwmL);
    rightMotor.rotateCW(pwmR);
  } else {
    leftMotor.rotateCW(pwmL);
    rightMotor.rotateCCW(pwmR);
  }
}

void backward(uint8_t pwmL, uint8_t pwmR){
  if (orientation == IMU_FACE_UP) {
    leftMotor.rotateCW(pwmL);
    rightMotor.rotateCCW(pwmR);
  } else {
    leftMotor.rotateCCW(pwmL);
    rightMotor.rotateCW(pwmR);
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

void controlMotorSpeedsForTurning() {
  leftMotor.Setpoint = 0.2;
  rightMotor.Setpoint = 0.2;
  while(1){      
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
  pid_setpoint = 0;
  Kp=1; Ki=0.1; Kd=0;
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
      forward(pwm_straight_drive, pwm_straight_drive + abs(pid_output));
    } else {
      forward(pwm_straight_drive + abs(pid_output), pwm_straight_drive);
      // leftpwm = target_pwm + abs(pid_output);
      // rightpwm = target_pwm;
    }
    // prev_error = pid_input; 
    rtos::ThisThread::sleep_for(2ms);
  }
}

void turnInPlaceByMag(float targetMag, uint8_t pwm) {
  // to ensure that the target is within the range of atan
  targetMag = targetMag - (int)(targetMag / (3.14/2)) * 3.14/2;

  spinCCW(30, 25);

  while (!closeTo(atan(imu.my / imu.mx), targetMag)) {
    rtos::ThisThread::sleep_for(5ms);
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
  if (orientation == IMU_FACE_DOWN) {
    leftMotor.rotateCW(pwmL);
    rightMotor.rotateCW(pwmR);
  } else {
    leftMotor.rotateCCW(pwmL);
    rightMotor.rotateCCW(pwmR);
  }
}

void spinCW(uint8_t pwmL, uint8_t pwmR) {
  if (orientation == IMU_FACE_DOWN) {
    leftMotor.rotateCCW(pwmL);
    rightMotor.rotateCCW(pwmR);
  } else {
    leftMotor.rotateCW(pwmL);
    rightMotor.rotateCW(pwmR);
  }
}

void driveToPole() {
  pid_setpoint = 1.5;
  Kp=1; Ki=0; Kd=0;
  while(1){
    pid_input = tofMatch; // 0,1,2 or 3
    // 0 means pole is right of middle
    // 1 or 2 mean that the pole is relatively in the middle
    // 3 means that pole is left of middle

    straightDrivePID.Compute();

    if (pid_input == 0) { // too far right, speed up left wheel
      forward(pwm_straight_drive + abs(pid_output), pwm_straight_drive);
    } else if (pid_input == 3) { // too far left, speed up right wheel
      forward(pwm_straight_drive, pwm_straight_drive + abs(pid_output));
    }    

    rtos::ThisThread::sleep_for(10ms);  
  }
}



