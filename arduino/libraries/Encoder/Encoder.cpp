/*
  Encoder.cpp
  associated .cpp file for the Encoder example class
*/

#include "Arduino.h"
#include "Encoder.h"

//default constructor
Encoder::Encoder(int channelA, int channelB, int countableEventsPerRev)
{

  //initialize pins
  setChannelAPin(channelA);
  setChannelBPin(channelB);
  pinMode(_channelAPin, INPUT);
  pinMode(_channelBPin, INPUT);

  //activate pull-up resistors
  digitalWrite(_channelAPin, HIGH);
  digitalWrite(_channelBPin, HIGH);

  //initialize other variables
  setCountableEventsPerRev(countableEventsPerRev);

  //initialize counts
  resetCounts();

  //initialize timer
  _timer.restart();

}

//basic functions
int Encoder::getChannelAPin()
{
  return _channelAPin;
}

int Encoder::getChannelBPin()
{
  return _channelBPin;
}

long Encoder::getChannelACount()
{
  return _countChannelA;
}

long Encoder::getChannelBCount()
{
  return _countChannelB;
}

void Encoder::resetChannelACount()
{
  _countChannelA = 0L;
}

void Encoder::resetChannelBCount()
{
  _countChannelB = 0L;
}

void Encoder::resetCounts()
{
  resetChannelACount();
  resetChannelBCount();
}

void Encoder::setChannelAPin(int pin)
{
  _channelAPin = pin;
}

void Encoder::setChannelBPin(int pin)
{
  _channelBPin = pin;
}

void Encoder::setCountableEventsPerRev(int value)
{
  _countableEventsPerRev = value;
}

//advanced functions

//returns current motor speed in radians per second
double Encoder::getRadPerSec()
{

  //get elapsed time since last time restart, then restart timer
  double secondsElapsed = _timer.elapsed() / 1000;
  _timer.restart();

  //get channel A counts since last count restart, then restart counts
  long counts = getChannelACount();
  resetCounts();

  //calculate and return motor speed in radians per second as double
  double radians = (counts / _countableEventsPerRev) * 2 * PI;
  return double(radians / secondsElapsed);

}

//returns current motor speed in revolutions per minute
double Encoder::getRPM()
{

    //get elapsed time since last time restart, then restart timer
    double minutesElapsed = _timer.elapsed() / 60000;
    _timer.restart();

    //get channel A counts since last count restart, then restart counts
    long counts = getChannelACount();
    resetCounts();

    //calculate and return motor speed in radians per second as double
    double revs = (counts / _countableEventsPerRev);
    return double(revs / minutesElapsed);

}

//increment encoder channel A count
void Encoder::addChannelACount()
{
  _countChannelA++;
}

//increment encoder channel B count
void Encoder::addChannelBCount()
{
  _countChannelB++;
}

//reset channel counts and timer to prevent overflow errors/crashes
void Encoder::preventOverflow()
{
  resetCounts();
  _timer.restart();
}
