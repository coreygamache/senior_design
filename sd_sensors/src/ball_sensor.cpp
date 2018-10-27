//includes
#include <ball_sensor.hpp>
//#include <ros/ros.h>
#include <wiringPi.h>

//default constructor
BallSensor::BallSensor(int output)
{

  //run wiringPi setup function
  wiringPiSetup();

  //set output pin
  setOutputPin(output);

}

//basic functions

//return output pin
int BallSensor::getOutputPin()
{
  return this->_outputPin;
}

//set output pin to given value
void BallSensor::setOutputPin(int output)
{
  this->_outputPin = output;
}

//advanced functions

//returns true if a ball is detected, and false otherwise
bool BallSensor::ballDetected()
{

  //if sensor output is high then ball is detected, else return false
  if (digitalRead(this->_outputPin))
    return true;
  else
    return false;

}
