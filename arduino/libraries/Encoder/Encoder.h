/*
  Encoder.h
  associated .h file for the Encoder class
*/

#ifndef Encoder_h
#define Encoder_h

//includes
#include "Arduino.h"
#include "Chrono.h"

//create Encoder class
class Encoder
{

  //protected properties
  protected:
    Chrono _timer;                //counts elapsed time in ms
    float _countableEventsPerRev;
    int _channelAPin;
    int _channelBPin;
    long _countChannelA;
    long _countChannelB;


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
    void resetChannelACount();
    void resetChannelBCount();
    void resetCounts();
    void setChannelAPin(int pin);
    void setChannelBPin(int pin);
    void setCountableEventsPerRev(float value);

    //advanced functions
    double getRadPerSec();
    double getRPM();
    void addChannelACount();
    void addChannelBCount();
    void preventOverflow();

};

#endif
