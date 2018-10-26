//includes
#include <ball_sensor.hpp>
//#include <ros/ros.h>
#include <wiringPi.h>

//default constructor
BallSensor(int output)
{

  //run wiringPi setup function
  wiringPiSetup();

  //set output pin
  setOutputPin(echo);

}

//basic functions

//return output pin
int getOutputPin()
{
  return this->_outputPin;
}

//set output pin to given value
void setOutputPin(int output)
{
  this->_outputPin = output;
}

//advanced functions

//returns true if a ball is detected, and false otherwise
bool ballDetected()
{

  //if sensor output is high then ball is detected, else return false
  if (digitalRead(this->_outputPin))
    return true;
  else
    return false;

}
