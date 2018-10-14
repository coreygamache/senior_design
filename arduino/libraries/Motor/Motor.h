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

  //protected properties
  protected:
    float _gearRatio;   //ratio of motor shaft rotations to output shaft rotations
    int _maxRPM;        //maximum output shaft RPM
    int _PWMPin;        //pin for outputting PWM signal
    int _sleepPin;      //pin for outputting sleep signal

  public:
    float getGearRatio();
    int getMaxRPM();
    int getPWMPin();
    int getSleepPin();
    void setGearRatio(float value);
    void setMaxRPM(int value);
    void setSleepPin(int pin);
    void setPWMPin(int pin);
    virtual void forward();
    virtual void forward(int pwmValue);
    virtual void reverse();
    virtual void reverse(int pwmValue);
    virtual void stop();

};

#endif
