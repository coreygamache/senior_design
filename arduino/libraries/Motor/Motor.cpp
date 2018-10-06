/*
  Motor.cpp
  associated .cpp file for the Motor class
*/

//includes
#include "Arduino.h"
#include "Motor.h"

//basic functions

//return gear ratio
float Motor::getGearRatio()
{
  return _gearRatio;
}

//return max RPM of motor
int Motor::getMaxRPM()
{
  return _maxRPM;
}

//return PWM pin
int Motor::getPWMPin()
{
  return _PWMPin;
}


//return sleep pin
int Motor::getSleepPin()
{
  return _sleepPin;
}

//set gear ratio
void Motor::setGearRatio(float value)
{
  _gearRatio = value;
}

//set max RPM of motor
void Motor::setMaxRPM(int value)
{
  _maxRPM = value;
}

//set PWM pin
void Motor::setPWMPin(int pin)
{
  _PWMPin = pin;
}


//set reverse pin
void Motor::setSleepPin(int pin)
{
  _sleepPin = pin;
}
