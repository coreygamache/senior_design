/*
  Encoder.h
  associated .h file for the Encoder class
*/

#ifndef Encoder_h
#define Encoder_h

//includes
#include "Arduino.h"

//create Encoder class
class Encoder
{

  //protected properties
  protected:
    float _countableEventsPerRev;
    int _channelAPin;
    int _channelBPin;
    unsigned long _lastStartTime;
    volatile long _countChannelA;
    volatile long _countChannelB;


  //public properties
  public:

    //constructors
    Encoder();
    Encoder(int channelA, int channelB, float countableEventsPerRev);

    //basic functions
    float getCountableEventsPerRev();
    int getChannelAPin();
    int getChannelBPin();
    long getChannelACount();
    long getChannelBCount();
    unsigned long getElapsedTime();
    void setChannelAPin(int pin);
    void setChannelBPin(int pin);
    void setChannelACount(long count);
    void setChannelBCount(long count);
    void setCountableEventsPerRev(float value);

    //advanced functions
    double getMotorRadPerSec();
    double getMotorRPM();
    void addChannelACount();
    void addChannelBCount();
    void reset();
    void resetChannelACount();
    void resetChannelBCount();
    void resetCounts();
    void resetTimer();

};

#endif
