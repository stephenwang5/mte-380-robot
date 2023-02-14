#include "encoder.h"

void Encoder::encoderUpdate(int motorDir) {
  if(motorDir == CW){
    encoderPos++;
  } 
  else if (motorDir == CCW) {
    encoderPos--;
  }
}