#include "motor.h"

#include "main.h"

int target_turn_pwm = 19;
// Variables for general PID used to match encoder ticks from both motors
double pid_setpoint = 0; double pid_output, pid_input;
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
  while(1){
    if (throwbotState == DRIVE) {

      // should get speed when it is needed rather than putting it in it's own thread
      //leftMotor.calculateSpeed();
      //rightMotor.calculateSpeed();
      
    //Run the PID calculation
    leftMotor.pidController.Compute();
    rightMotor.pidController.Compute();

    //forward(leftMotor.Output, rightMotor.Output);
    leftMotor.rotateCCW(target_turn_pwm+leftMotor.Output);
    rightMotor.rotateCCW(target_turn_pwm+rightMotor.Output);
    }
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

  uint8_t target_pwm = 50; 
  while(1) {
    if (throwbotState == DRIVE){
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
    } 
    rtos::ThisThread::sleep_for(2ms);
  }
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




