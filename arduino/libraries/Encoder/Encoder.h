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

  //private properties
  private:
    Chrono _timer;                //counts elapsed time in ms
    int _channelAPin;
    int _channelBPin;
    int _countableEventsPerRev;
    long _countChannelA;
    long _countChannelB;


  //public properties
  public:

    //basic functions
    Encoder(int channelA, int channelB, int countableEventsPerRev);
    int getChannelAPin();
    int getChannelBPin();
    long getChannelACount();
    long getChannelBCount();
    void resetChannelACount();
    void resetChannelBCount();
    void resetCounts();
    void setChannelAPin(int pin);
    void setChannelBPin(int pin);
    void setCountableEventsPerRev(int value);

    //advanced functions
    double getRadPerSec();
    double getRPM();
    void addChannelACount();
    void addChannelBCount();
    void preventOverflow();

};

#endif
