/*
  Motor.cpp
  associated .cpp file for the Motor class
*/

//includes
#include "Motor.h"

//constructor
Motor::Motor() {}

//basic functions

//return gear ratio
float Motor::getGearRatio()
{
  return this->_gearRatio;
}

//return max RPM of motor
int Motor::getMaxRPM()
{
  return this->_maxRPM;
}

//return PWM pin
int Motor::getPWMPin()
{
  return this->_PWMPin;
}


//return sleep pin
int Motor::getSleepPin()
{
  return this->_sleepPin;
}

//set gear ratio
void Motor::setGearRatio(float value)
{
  this->_gearRatio = value;
}

//set max RPM of motor
void Motor::setMaxRPM(int value)
{
  this->_maxRPM = value;
}

//set PWM pin
void Motor::setPWMPin(int pin)
{
  this->_PWMPin = pin;
}


//set reverse pin
void Motor::setSleepPin(int pin)
{
  this->_sleepPin = pin;
}
