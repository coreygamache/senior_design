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
    int _fwdPin;
    int _maxRPM;
    int _revPin;

  public:
    Motor(int fPin, int rPin, int maxRPM);
    int getFwdPin();
    int getMaxRPM();
    int getRevPin();
    void setFwdPin(int pin);
    void setMaxRPM(int value);
    void setRevPin(int pin);
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
