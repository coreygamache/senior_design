/*
  DriveMotor.h
  associated .h file for the DriveMotor class
*/

#ifndef DriveMotor_h
#define DriveMotor_h

//includes
#include "Arduino.h"
#include "Motor.h"

//create DriveMotor class inheriting Motor class
class DriveMotor : public Motor
{

  //private properties
  private:
    int _directionPin;        //LOW for forward direction, HIGH for reverse direction

  //public properties
  public:

    //constructors
    DriveMotor();
    DriveMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM);

    //basic functions
    int getDirectionPin();
    void setDirectionPin(int pin);

    //advanced functions
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
