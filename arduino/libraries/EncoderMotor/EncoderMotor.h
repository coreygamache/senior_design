/*
  EncoderMotor.h
  associated .h file for the EncoderMotor class
*/

#ifndef EncoderMotor_h
#define EncoderMotor_h

//includes
#include "Arduino.h"
#include "ComponentMotor.h"
#include "DriveMotor.h"
#include "Encoder.h"

//create EncoderMotor class inheriting Encoder class
class EncoderMotor : public Encoder
{

  //private properties
  private:
    bool _isDriveMotor;
    ComponentMotor _componentMotor = ComponentMotor();
    DriveMotor _driveMotor = DriveMotor();

  //public properties
  public:

    //default constructor
    EncoderMotor();

    //component motor constructor
    EncoderMotor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev);
    EncoderMotor(ComponentMotor cMotor, Encoder encoder);

    //drive motor constructor
    EncoderMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev);
    EncoderMotor(DriveMotor dMotor, Encoder encoder);

    //basic functions
    ComponentMotor getComponentMotor();
    DriveMotor getDriveMotor();
    Encoder getEncoder();
    void setEncoder(Encoder encoder);
    void setIsDriveMotor(bool driveMotor);
    void setMotor(ComponentMotor motor);
    void setMotor(DriveMotor motor);

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
