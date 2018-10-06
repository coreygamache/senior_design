/*
  DriveMotor.h
  associated .h file for the DriveMotor class
*/

#ifndef DriveMotor_h
#define DriveMotor_h

//includes
#include "Arduino.h"
#include "Motor.h"

//create DriveMotor class
class DriveMotor
{

  //private properties
  private:
    int _directionPin;        //when LOW current flows A to B and vice versa

  //public properties
  public:
    Motor();
    Motor(int dir, int pwm, int sleep, float gearRatio, int maxRPM);
    int getDirectionPin();
    void setDirectionPin(int pin);
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
