/*
  DriveMotor.cpp
  associated .cpp file for the DriveMotor class
*/

//includes
#include "Arduino.h"
#include "DriveMotor.h"

//basic functions

//default constructorS

Motor::Motor()
{
  setDirectionPin(-1);
  setPWMPin(-1);
  setSleepPin(-1);
  setGearRatio(0);
  setMaxRPM(0);
}

Motor::Motor(int dir, int pwm, int sleep, float gearRatio, int maxRPM)
{

  //initialize pins
  setDirectionPin(dir);
  setPWMPin(pwm);
  setSleepPin(sleep);

  //set pin modes
  pinMode(_directionPin, OUTPUT);
  pinMode(_PWMPin, OUTPUT);
  pinMode(_sleepPin, OUTPUT);

  //initialize other variables
  setGearRatio(gearRatio);
  setMaxRPM(maxRPM);

}

//return forward pin
int Motor::getDirectionPin()
{
  return _directionPin;
}

//set forward pin
void Motor::setDirectionPin(int pin)
{
  _directionPin = pin;
}

//advanced functions

//set motor to rotate forward at maximum speed indefinitely
void Motor::forward()
{
  digitalWrite(_directionPin, LOW);
  digitalWrite(_PWMPin, HIGH);
}

//overloaded forward() function: set motor to rotate forward at speed specified by PWM value
void Motor::forward(int pwmValue)
{
  digitalWrite(_directionPin, LOW);
  analogWrite(_PWMPin, pwmValue);
}

//set motor to rotate in reverse at maximum speed indefinitely
void Motor::reverse()
{
  digitalWrite(_directionPin, HIGH);
  digitalWrite(_PWMPin, HIGH);
}

//overloaded reverse() function: set motor to rotate in reverse at speed specified by PWM value
void Motor::reverse(int pwmValue)
{
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
