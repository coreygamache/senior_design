/*
  Motor.cpp
  associated .cpp file for the Motor class
*/

//includes
#include "Arduino.h"
#include "Motor.h"

//basic functions

//default constructor
Motor::Motor(int fPin, int rPin, int maxRPM)
{
  pinMode(fPin, OUTPUT);
  pinMode(rPin, OUTPUT);
  setFwdPin(fPin);
  setMaxRPM(maxRPM);
  setRevPin(rPin);
}

//return forward pin
int Motor::getFwdPin()
{
  return _fwdPin;
}

//return max RPM of motor
int Motor::getMaxRPM()
{
  return _maxRPM;
}

//return reverse pin
int Motor::getRevPin()
{
  return _revPin;
}

//set forward pin
void Motor::setFwdPin(int pin)
{
  _fwdPin = pin;
}

//set max RPM of motor
void Motor::setMaxRPM(int value)
{
  _maxRPM = value;
}

//set reverse pin
void Motor::setRevPin(int pin)
{
  _revPin = pin;
}

//advanced functions

//set motor to rotate forward at maximum speed indefinitely
void Motor::forward()
{
  digitalWrite(_revPin, LOW);
  digitalWrite(_fwdPin, HIGH);
}

//overloaded forward() function: set motor to rotate forward at speed specified by PWM value
void Motor::forward(int pwmValue)
{
  digitalWrite(_revPin, LOW);
  analogWrite(_fwdPin, pwmValue);
}

//set motor to rotate in reverse at maximum speed indefinitely
void Motor::reverse()
{
  digitalWrite(_fwdPin, LOW);
  digitalWrite(_revPin, HIGH);
}

//overloaded reverse() function: set motor to rotate in reverse at speed specified by PWM value
void Motor::reverse(int pwmValue)
{
  digitalWrite(_fwdPin, LOW);
  analogWrite(_revPin, pwmValue);
}

//stop the motor
void Motor::stop()
{
  digitalWrite(_fwdPin, LOW);
  digitalWrite(_revPin, LOW);
}
