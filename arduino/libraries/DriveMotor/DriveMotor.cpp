/*
  DriveMotor.cpp
  associated .cpp file for the DriveMotor class
*/

//includes
#include "DriveMotor.h"

//default constructorS

DriveMotor::DriveMotor()
{
  this->setDirectionPin(-1);
  this->setPWMPin(-1);
  this->setSleepPin(-1);
  this->setGearRatio(0);
  this->setMaxRPM(0);
}

DriveMotor::DriveMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM)
{

  //initialize pins
  this->setDirectionPin(dir);
  this->setPWMPin(pwm);
  this->setSleepPin(sleep);

  //set pin modes
  pinMode(this->_directionPin, OUTPUT);
  pinMode(this->_PWMPin, OUTPUT);
  pinMode(this->_sleepPin, OUTPUT);

  //turn off sleep mode by default
  digitalWrite(this->_sleepPin, HIGH);

  //initialize other variables
  this->setGearRatio(gearRatio);
  this->setMaxRPM(maxRPM);

}

//basic functions

//return forward pin
int DriveMotor::getDirectionPin()
{
  return this->_directionPin;
}

//set forward pin
void DriveMotor::setDirectionPin(int pin)
{
  this->_directionPin = pin;
}

//advanced functions

//set motor to rotate forward at maximum speed indefinitely
void DriveMotor::forward()
{
  digitalWrite(this->_directionPin, LOW);
  digitalWrite(this->_PWMPin, HIGH);
}

//overloaded forward() function: set motor to rotate forward at speed specified by PWM value
void DriveMotor::forward(int pwmValue)
{
  digitalWrite(this->_directionPin, LOW);
  analogWrite(this->_PWMPin, pwmValue);
}

//set motor to rotate in reverse at maximum speed indefinitely
void DriveMotor::reverse()
{
  digitalWrite(this->_directionPin, HIGH);
  digitalWrite(this->_PWMPin, HIGH);
}

//overloaded reverse() function: set motor to rotate in reverse at speed specified by PWM value
void DriveMotor::reverse(int pwmValue)
{
  digitalWrite(this->_directionPin, HIGH);
  analogWrite(this->_PWMPin, pwmValue);
}

//put motor controller into sleep mode
void DriveMotor::sleep()
{
  digitalWrite(this->_sleepPin, LOW);
}

//stop the motor
void DriveMotor::stop()
{
  digitalWrite(this->_PWMPin, LOW);
}

//take motor controller out of sleep mode
void DriveMotor::wakeUp()
{
  digitalWrite(this->_sleepPin, LOW);
}
