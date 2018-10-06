/*
  ComponentMotor.cpp
  associated .cpp file for the ComponentMotor class
*/

//includes
#include "ComponentMotor.h"

//default constructors

ComponentMotor::ComponentMotor()
{
  this->setInputAPin(-1);
  this->setInputBPin(-1);
  this->setPWMPin(-1);
  this->setSleepPin(-1);
  this->setGearRatio(0);
  this->setMaxRPM(0);
}

ComponentMotor::ComponentMotor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM)
{

  //initialize pins
  this->setInputAPin(inputA);
  this->setInputBPin(inputB);
  this->setPWMPin(pwm);
  this->setSleepPin(sleep);

  //set pin modes
  pinMode(this->_inputAPin, OUTPUT);
  pinMode(this->_inputBPin, OUTPUT);
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
int ComponentMotor::getInputAPin()
{
  return this->_inputAPin;
}

//return reverse pin
int ComponentMotor::getInputBPin()
{
  return this->_inputBPin;
}

//set forward pin
void ComponentMotor::setInputAPin(int pin)
{
  this->_inputAPin = pin;
}

//set reverse pin
void ComponentMotor::setInputBPin(int pin)
{
  this->_inputBPin = pin;
}

//advanced functions

//set motor to rotate forward at maximum speed indefinitely
void ComponentMotor::forward()
{
  digitalWrite(this->_inputAPin, LOW);
  digitalWrite(this->_inputBPin, HIGH);
  digitalWrite(this->_PWMPin, HIGH);
}

//overloaded forward() function: set motor to rotate forward at speed specified by PWM value
void ComponentMotor::forward(int pwmValue)
{
  digitalWrite(this->_inputAPin, LOW);
  digitalWrite(this->_inputBPin, HIGH);
  analogWrite(this->_PWMPin, pwmValue);
}

//set motor to rotate in reverse at maximum speed indefinitely
void ComponentMotor::reverse()
{
  digitalWrite(this->_inputBPin, LOW);
  digitalWrite(this->_inputAPin, HIGH);
  digitalWrite(this->_PWMPin, HIGH);
}

//overloaded reverse() function: set motor to rotate in reverse at speed specified by PWM value
void ComponentMotor::reverse(int pwmValue)
{
  digitalWrite(this->_inputBPin, LOW);
  digitalWrite(this->_inputAPin, HIGH);
  analogWrite(this->_PWMPin, pwmValue);
}

//put motor controller into sleep mode
void ComponentMotor::sleep()
{
  digitalWrite(this->_sleepPin, LOW);
}

//stop the motor
void ComponentMotor::stop()
{
  digitalWrite(this->_inputAPin, LOW);
  digitalWrite(this->_inputBPin, LOW);
  digitalWrite(this->_PWMPin, LOW);
}


//take motor controller out of sleep mode
void ComponentMotor::wakeUp()
{
  digitalWrite(this->_sleepPin, LOW);
}
