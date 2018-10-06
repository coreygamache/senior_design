/*
  EncoderMotor.h
  associated .h file for the EncoderMotor class
*/

#ifndef EncoderMotor_h
#define EncoderMotor_h

//includes
#include "Arduino.h"
#include "Encoder.h"
#include "Motor.h"

//create EncoderMotor class
class EncoderMotor
{

  //private properties
  private:
    Encoder _encoder = Encoder();
    Motor _motor = Motor();

  public:

    //basic functions
    EncoderMotor(int fPin, int rPin, float gearRatio, int maxRPM, int channelA, int channelB, int countableEventsPerRev);
    Encoder getEncoder();
    Motor getMotor();
    void setEncoder(Encoder encoder);
    void setMotor(Motor motor);

    //advanced functions

    //encoder functions
    void addChannelACount();
    void addChannelBCount();
    void preventOverflow();

    //motor functions
    void forward();
    void forward(int pwmValue);
    void reverse();
    void reverse(int pwmValue);
    void stop();

    //encoder motor functions
    double getRadPerSec();
    double getRPM();

};

#endif
