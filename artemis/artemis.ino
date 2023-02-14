#include "motor.h"
#include "encoder.h"
#include <Wire.h>
#include "tof.h"
#include "imu.h"

#define I2C_FREQ 400000

const int robotWidth = 4; // robot width in inches
const int wheelRadius = 3; // wheel radius in inches
#define TICKS_PER_REV 2100 // ticks per revolution of the encoder

Motor L_Motor = Motor(35,6,Encoder(14)); //bridged pin 6 to pin 34 on board because pin 34 wouldnt output a PWM
Motor R_Motor = Motor(29,11,Encoder(23));

/********** ISRs **********/

static void RencoderUpdateISR() {
  R_Motor.encoder.encoderUpdate(R_Motor.direction);
}

static void LencoderUpdateISR() {
  L_Motor.encoder.encoderUpdate(L_Motor.direction);
}

/********** ISRs **********/

TwoWire i2c(D25, D27);
ToF tof;
IMU imu;

void setup() {

  Serial.begin(115200);
  while(!Serial);
  pinMode(34, INPUT); // set broken motor pin to input so it doesnt interfere with pwm at now at pin 6

  attachInterrupt(digitalPinToInterrupt(L_Motor.encoder.pinA), RencoderUpdateISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_Motor.encoder.pinA), LencoderUpdateISR, CHANGE);

  i2c.begin();
  i2c.setClock(I2C_FREQ);

  tof.begin();
  imu.begin(i2c, I2C_FREQ);

}

void forward(uint8_t pwm){
  // Left wheel needs to go CCW
  L_Motor.counterclockwise(pwm);

  // Right wheel needs to go CW
  R_Motor.clockwise(pwm);

}

void backward(uint8_t pwm){
  // Left wheel needs to go CW
  L_Motor.clockwise(pwm);

  // Right wheel needs to go CCW
  R_Motor.counterclockwise(pwm);

}

void coast(){
  L_Motor.coast();
  R_Motor.coast();
}

void TurnInPlace(int numDegrees) {

  int clockwise = numDegrees > 0;
  int pwm = 100;

  float turnDist = PI*12*(abs(numDegrees))/360.0f; // turn distance of each wheel
  uint16_t numticks = uint16_t((turnDist*TICKS_PER_REV)/(PI*2*wheelRadius)); // number of encoder ticks needed to turn provided number of degrees

  int R_startPos = R_Motor.encoder.encoderPos;
  int L_startPos = L_Motor.encoder.encoderPos;

  if (clockwise) { // Left wheel goes forward, right wheel goes back => both wheels spin CCW
    R_Motor.counterclockwise(pwm);
    L_Motor.counterclockwise(pwm);

  } else { // Right wheel goes forward, left wheel goes back => both wheels spin CW
    R_Motor.clockwise(pwm);
    L_Motor.clockwise(pwm);
  }
  int Rturning  = 1; int Lturning = 1;
  while (Rturning || Lturning) {
    if (abs(R_Motor.encoder.encoderPos - R_startPos) >= numticks) {
      R_Motor.coast();
      Rturning = 0;
    }
    if (abs(L_Motor.encoder.encoderPos - L_startPos) >= numticks) {
      L_Motor.coast();
      Lturning = 0;
    }
  }
}

void loop() {
  tof.read();
  Serial.print(tof.data.distance_mm[3]);
  Serial.println();
  delay(3);
  // // put your main code here, to run repeatedly:
  // TurnInPlace(-90);
  // delay(2000);
  // forward(100);
  // delay(2000);
  // coast();
  // delay(2000);
  // TurnInPlace(90);
  // delay(2000);
  // backward(100);
  // delay(2000);
  // coast();
  // delay(4000);
}
