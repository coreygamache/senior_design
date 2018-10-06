/*
  ComponentMotor.h
  associated .h file for the ComponentMotor class
*/

#ifndef ComponentMotor_h
#define ComponentMotor_h

//includes
#include "Arduino.h"
#include "Motor.h"

//create ComponentMotor class
class ComponentMotor
{

  //private properties
  private:
    int _inputAPin;
    int _inputBPin;

  //public properties
  public:
    Motor();
    Motor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM);
    int getInputAPin();
    int getInputBPin();
    void setInputAPin(int pin);
    void setInputBPin(int pin);
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
