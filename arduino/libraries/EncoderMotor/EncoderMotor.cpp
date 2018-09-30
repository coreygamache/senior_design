/*
  EncoderMotor.cpp
  associated .cpp file for the EncoderMotor class
*/

//includes
#include "Arduino.h"
#include "EncoderMotor.h"

//default constructor
EncoderMotor::EncoderMotor(int fPin, int rPin, int maxRPM, int channelA, int channelB, int countableEventsPerRev)
{
  _encoder = Encoder(channelA, channelB, countableEventsPerRev);
  _motor = Motor(fPin, rPin, maxRPM);
}

//basic function

Encoder EncoderMotor::getEncoder()
{
  return _encoder;
}

Motor EncoderMotor::getMotor()
{
  return _motor;
}

void EncoderMotor::setEncoder(Encoder encoder)
{
  _encoder = encoder;
}

void EncoderMotor::setMotor(Motor motor)
{
  _motor = motor;
}

//advanced functions

//encoder functions

//return motor speed in radians per second
double EncoderMotor::getRadPerSec(){
  return _encoder.getRadPerSec();
}

//return motor speed in revolutions per minute
double EncoderMotor::getRPM()
{
  return _encoder.getRPM();
}

//increment encoder channel A count
void EncoderMotor::addChannelACount()
{
  _encoder.addChannelACount();
}

//increment encoder channel B count
void EncoderMotor::addChannelBCount()
{
  _encoder.addChannelBCount();
}

//prevent overflow of encoder variables (THIS IS IMPORTANT)
void EncoderMotor::preventOverflow()
{
  _encoder.preventOverflow();
}

//motor functions

void EncoderMotor::forward()
{
    _motor.forward();
}

void EncoderMotor::forward(int pwmValue)
{
  _motor.forward(pwmValue);
}

void EncoderMotor::reverse()
{
  _motor.reverse();
}

void EncoderMotor::reverse(int pwmValue)
{
  _motor.reverse(pwmValue);
}

void EncoderMotor::stop()
{
  _motor.stop();
}
