/*
  ComponentMotor.cpp
  associated .cpp file for the ComponentMotor class
*/

//includes
#include "Arduino.h"
#include "ComponentMotor.h"

//basic functions

//default constructorS

Motor::Motor()
{
  setInputAPin(-1);
  setInputBPin(-1);
  setPWMPin(-1);
  setSleepPin(-1);
  setGearRatio(0);
  setMaxRPM(0);
}

Motor::Motor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM)
{

  //initialize pins
  setInputAPin(inputA);
  setInputBPin(inputB);
  setPWMPin(pwm);
  setSleepPin(sleep);

  //set pin modes
  pinMode(_inputAPin, OUTPUT);
  pinMode(_inputBPin, OUTPUT);
  pinMode(_PWMPin, OUTPUT);
  pinMode(_sleepPin, OUTPUT);

  //initialize other variables
  setGearRatio(gearRatio);
  setMaxRPM(maxRPM);

}

//return forward pin
int Motor::getInputAPin()
{
  return _inputAPin;
}

//return reverse pin
int Motor::getInputBPin()
{
  return _inputBPin;
}

//set forward pin
void Motor::setInputAPin(int pin)
{
  _inputAPin = pin;
}

//set reverse pin
void Motor::setInputBPin(int pin)
{
  _inputBPin = pin;
}

//advanced functions

//set motor to rotate forward at maximum speed indefinitely
void Motor::forward()
{
  digitalWrite(_inputAPin, LOW);
  digitalWrite(_inputBPin, HIGH);
  digitalWrite(_PWMPin, HIGH);
}

//overloaded forward() function: set motor to rotate forward at speed specified by PWM value
void Motor::forward(int pwmValue)
{
  digitalWrite(_inputAPin, LOW);
  digitalWrite(_inputBPin, HIGH);
  analogWrite(_PWMPin, pwmValue);
}

//set motor to rotate in reverse at maximum speed indefinitely
void Motor::reverse()
{
  digitalWrite(_inputBPin, LOW);
  digitalWrite(_inputAPin, HIGH);
  digitalWrite(_PWMPin, HIGH);
}

//overloaded reverse() function: set motor to rotate in reverse at speed specified by PWM value
void Motor::reverse(int pwmValue)
{
  digitalWrite(_inputBPin, LOW);
  digitalWrite(_inputAPin, HIGH);
  analogWrite(_PWMPin, pwmValue);
}

//stop the motor
void Motor::stop()
{
  digitalWrite(_inputAPin, LOW);
  digitalWrite(_inputBPin, LOW);
  digitalWrite(_PWMPin, LOW);
}
