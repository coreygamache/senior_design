/*
  EncoderMotor.cpp
  associated .cpp file for the EncoderMotor class
*/

//includes
#include "Arduino.h"
#include "EncoderMotor.h"

//default constructor
EncoderMotor::EncoderMotor(int fPin, int rPin, float gearRatio, int maxRPM, int channelA, int channelB, int countableEventsPerRev)
{
  _encoder = Encoder(channelA, channelB, countableEventsPerRev);
  _motor = Motor(fPin, rPin, gearRatio, maxRPM);
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

//encoder motor functions

//return motor output shaft speed in radians per second
//this value is calculated taking into account any reduction gearing
double EncoderMotor::getRadPerSec(){
  double radPerSec_motor = _encoder.getRadPerSec();
  double radPerSec_output = radPerSec_motor / double(_motor.getGearRatio());
  return radPerSec_output;
}

//return motor output shaft speed in revolutions per minute
//this value is calculated taking into account any reduction gearing
double EncoderMotor::getRPM()
{
  double RPM_motor = _encoder.getRPM();
  double RPM_output = RPM_motor / double(_motor.getGearRatio());
  return RPM_output;
}
