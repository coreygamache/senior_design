/*
  Motor.h
  associated .h file for the Motor class
*/

#ifndef Motor_h
#define Motor_h

//includes
#include "Arduino.h"

//create Motor class
class Motor
{

  //private properties
  private:
    float _gearRatio;   //ratio of motor shaft rotations to output shaft rotations
    int _fwdPin;
    int _maxRPM;        //maximum output shaft RPM
    int _revPin;

  public:
    Motor(int fPin, int rPin, float gearRatio, int maxRPM);
    float getGearRatio();
    int getFwdPin();
    int getMaxRPM();
    int getRevPin();
    void setFwdPin(int pin);
    void setGearRatio(float value);
    void setMaxRPM(int value);
    void setRevPin(int pin);
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
