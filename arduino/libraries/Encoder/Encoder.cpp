/*
  Encoder.cpp
  associated .cpp file for the Encoder example class
*/

#include "Encoder.h"

//default constructor
Encoder::Encoder()
{

  //initialize pins
  this->setChannelAPin(-1);
  this->setChannelBPin(-1);

  //initalize other variables
  this->setCountableEventsPerRev(0);

  //initialize counts
  this->reset();

}

//overloaded constructor
Encoder::Encoder(int channelA, int channelB, float countableEventsPerRev)
{

  //initialize pins
  this->setChannelAPin(channelA);
  this->setChannelBPin(channelB);

  //set pin modes
  pinMode(this->_channelAPin, INPUT);
  pinMode(this->_channelBPin, INPUT);

  //activate pull-up resistors
  digitalWrite(this->_channelAPin, HIGH);
  digitalWrite(this->_channelBPin, HIGH);

  //initialize other variables
  this->setCountableEventsPerRev(countableEventsPerRev);

  //initialize counts
  this->reset();

}

//basic functions
float Encoder::getCountableEventsPerRev()
{
  return this->_countableEventsPerRev;
}

int Encoder::getChannelAPin()
{
  return this->_channelAPin;
}

int Encoder::getChannelBPin()
{
  return this->_channelBPin;
}

long Encoder::getChannelACount()
{
  return this->_countChannelA;
}

long Encoder::getChannelBCount()
{
  return this->_countChannelB;
}

unsigned long Encoder::getElapsedTime()
{
  return (millis() - this->_lastStartTime);
}

void Encoder::setChannelAPin(int pin)
{
  this->_channelAPin = pin;
}

void Encoder::setChannelBPin(int pin)
{
  this->_channelBPin = pin;
}

void Encoder::setChannelACount(long count)
{
  if ((count >= 0) && (count < 2147483647))
    this->_countChannelA = count;
}

void Encoder::setChannelBCount(long count)
{
  if ((count >= 0) && (count < 2147483647))
    this->_countChannelB = count;
}

void Encoder::setCountableEventsPerRev(float value)
{
  this->_countableEventsPerRev = value;
}

//advanced functions

//returns current motor speed in radians per second
double Encoder::getMotorRadPerSec()
{

  //get elapsed time since last time restart
  double secondsElapsed = double(this->getElapsedTime()) / 1000;

  //get channel A counts since last count restart
  long counts = this->getChannelACount();

  //calculate and return motor speed in radians per second as double
  double radians = (counts / this->getCountableEventsPerRev()) * 2 * PI;
  return (radians / secondsElapsed);

}

//returns current motor speed in revolutions per minute
double Encoder::getMotorRPM()
{

    //get elapsed time since last time restart
    double secondsElapsed = double(this->getElapsedTime()) / 1000;

    //get channel A counts since last count restart
    long counts = this->getChannelACount();

    //calculate and return motor speed in radians per second as double
    double revs = (counts / this->getCountableEventsPerRev());
    return ((revs / secondsElapsed) * 60);

}

//increment encoder channel A count
void Encoder::addChannelACount()
{

  //if count will overflow then reset encoder properties to prevent overflow before iterating
  if (this->_countChannelA == 2147483647)
    this->reset();
  this->_countChannelA++;

}

//increment encoder channel B count
void Encoder::addChannelBCount()
{

  //if count will overflow then reset encoder properties to prevent overflow before iterating
  if (this->_countChannelB == 2147483647)
    this->reset();
  this->_countChannelB++;

}

//reset channel counts and timer
void Encoder::reset()
{
  this->resetTimer();
  this->resetCounts();
}

void Encoder::resetChannelACount()
{
  this->_countChannelA = 0L;
}

void Encoder::resetChannelBCount()
{
  this->_countChannelB = 0L;
}

void Encoder::resetCounts()
{
  this->resetChannelACount();
  this->resetChannelBCount();
}

void Encoder::resetTimer()
{
  this->_lastStartTime = millis();
}
