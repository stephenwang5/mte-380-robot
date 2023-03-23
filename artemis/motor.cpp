#include "motor.h"

#include "main.h"

Motor::Motor(PinName a, PinName b, PinName enc): pinA(a), pinB(b), encoderPin(enc), pidController(&speed, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {}

void Motor::begin() {
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(encoderPin, INPUT);
  straightLinePID.SetOutputLimits(-255 + target_pwm, 255 - target_pwm);
  straightLinePID.SetMode(AUTOMATIC);
}

void Motor::rotateCW(uint8_t pwm) {
  analogWrite(pinA, pwm);
  analogWrite(pinB, 0);
  direction = CW;
}

void Motor::rotateCCW(uint8_t pwm) {
  analogWrite(pinA, 0);
  analogWrite(pinB, pwm);
  direction = CCW;
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

void controlMotorSpeeds() {
  while(1){

    // should get speed when it is needed rather than putting it in it's own thread
    leftMotor.calculateSpeed();
    rightMotor.calculateSpeed();

  //Run the PID calculation
  leftMotor.pidController.Compute();
  rightMotor.pidController.Compute();

  forward(leftMotor.Output, rightMotor.Output);      
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

void controlMotorSpeedsWithEncoderCount() {
  while(1) {

    pid_input = abs(leftMotor.encoder) - rightMotor.encoder;
    // acc_error += pid_input;
    
    // P = pid_input * Kp;
    // I = acc_error * Ki * 0.001;
    // D = Kd * (pid_input - prev_error) / 0.001;

    //Run the PID calculation
    straightLinePID.Compute();

    if (pid_input > 0){
      leftpwm = target_pwm; 
      rightpwm = target_pwm + abs(pid_output);
      forward(target_pwm, target_pwm + abs(pid_output));
    } else {
      forward(target_pwm + abs(pid_output), target_pwm);
      leftpwm = target_pwm + abs(pid_output);
      rightpwm = target_pwm;
    }
    // prev_error = pid_input;
    rtos::ThisThread::sleep_for(2ms);
  }
}




