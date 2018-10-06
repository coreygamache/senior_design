/*
  ComponentMotor.h
  associated .h file for the ComponentMotor class
*/

#ifndef ComponentMotor_h
#define ComponentMotor_h

//includes
#include "Arduino.h"
#include "Motor.h"

//create ComponentMotor class inheriting motor class
class ComponentMotor : public Motor
{

  //private properties
  private:
    int _inputAPin;
    int _inputBPin;

  //public properties
  public:

    //constructors
    ComponentMotor();
    ComponentMotor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM);

    //basic functions
    int getInputAPin();
    int getInputBPin();
    void setInputAPin(int pin);
    void setInputBPin(int pin);

    //advanced functions
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

};

#endif
