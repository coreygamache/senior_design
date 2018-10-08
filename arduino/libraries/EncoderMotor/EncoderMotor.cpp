/*
  EncoderMotor.cpp
  associated .cpp file for the EncoderMotor class
*/

//includes
#include "EncoderMotor.h"

//default constructor
EncoderMotor::EncoderMotor()
{

  //initialize pins
  this->setChannelAPin(-1);
  this->setChannelBPin(-1);

  //initalize other variables
  this->setCountableEventsPerRev(0);
  this->setIsDriveMotor(false);

  //initialize counts
  this->resetCounts();

}

//component motor constructor
EncoderMotor::EncoderMotor(int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev)
{

  //initialize motor object and specify motor type
  this->setMotor(ComponentMotor(inputA, inputB, pwm, sleep, gearRatio, maxRPM));
  this->setIsDriveMotor(false);

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
  this->resetCounts();

  //initialize timer
  this->_timer.restart();

}

//component motor constructor via objects
EncoderMotor::EncoderMotor(ComponentMotor cMotor, Encoder encoder)
{

  //initialize motor object and specify motor type
  this->setMotor(cMotor);
  this->setIsDriveMotor(false);

  //initialize pins
  this->setChannelAPin(encoder.getChannelAPin());
  this->setChannelBPin(encoder.getChannelBPin());

  //set pin modes
  pinMode(this->_channelAPin, INPUT);
  pinMode(this->_channelBPin, INPUT);

  //activate pull-up resistors
  digitalWrite(this->_channelAPin, HIGH);
  digitalWrite(this->_channelBPin, HIGH);

  //initialize other variables
  this->setCountableEventsPerRev(encoder.getCountableEventsPerRev());

  //initialize counts
  this->resetCounts();

  //initialize timer
  this->_timer.restart();

}

//drive motor constructor
EncoderMotor::EncoderMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev)
{

  //initialize motor object and specify motor type
  this->setMotor(DriveMotor(dir, pwm, sleep, gearRatio, maxRPM));
  this->setIsDriveMotor(true);

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
  this->resetCounts();

  //initialize timer
  this->_timer.restart();

}

//component motor constructor via objects
EncoderMotor::EncoderMotor(DriveMotor dMotor, Encoder encoder)
{

  //initialize motor object and specify motor type
  this->setMotor(dMotor);
  this->setIsDriveMotor(true);

  //initialize pins
  this->setChannelAPin(encoder.getChannelAPin());
  this->setChannelBPin(encoder.getChannelBPin());

  //set pin modes
  pinMode(this->_channelAPin, INPUT);
  pinMode(this->_channelBPin, INPUT);

  //activate pull-up resistors
  digitalWrite(this->_channelAPin, HIGH);
  digitalWrite(this->_channelBPin, HIGH);

  //initialize other variables
  this->setCountableEventsPerRev(encoder.getCountableEventsPerRev());

  //initialize counts
  this->resetCounts();

  //initialize timer
  this->_timer.restart();

}

//basic functions

ComponentMotor EncoderMotor::getComponentMotor()
{
  return this->_componentMotor;
}

DriveMotor EncoderMotor::getDriveMotor()
{
  return this->_driveMotor;
}

Encoder EncoderMotor::getEncoder()
{
  return Encoder(this->_channelAPin, this->_channelBPin, this->_countableEventsPerRev);
}

void EncoderMotor::setIsDriveMotor(bool driveMotor)
{
  this->_isDriveMotor = driveMotor;
}

void EncoderMotor::setMotor(ComponentMotor motor)
{
  this->_componentMotor = motor;
  this->setIsDriveMotor(false);
}

void EncoderMotor::setMotor(DriveMotor motor)
{
  this->_driveMotor = motor;
  this->setIsDriveMotor(true);
}

//motor functions
//each function calls the relevant function for the type of motor associated with this instance of the EncoderMotor object

void EncoderMotor::forward()
{
  if (this->_isDriveMotor)
    this->_driveMotor.forward();
  else
    this->_componentMotor.forward();
}

void EncoderMotor::forward(int pwmValue)
{
  if (this->_isDriveMotor)
    this->_driveMotor.forward(pwmValue);
  else
    this->_componentMotor.forward(pwmValue);
}

void EncoderMotor::reverse()
{
  if (this->_isDriveMotor)
    this->_driveMotor.reverse();
  else
    this->_componentMotor.reverse();
}

void EncoderMotor::reverse(int pwmValue)
{
  if (this->_isDriveMotor)
    this->_driveMotor.reverse(pwmValue);
  else
    this->_componentMotor.reverse(pwmValue);
}

void EncoderMotor::stop()
{
  if (this->_isDriveMotor)
    this->_driveMotor.stop();
  else
    this->_componentMotor.stop();
}

//encoder motor functions

//return motor output shaft speed in radians per second
//this value is calculated taking into account any reduction gearing
double EncoderMotor::getOutputRadPerSec(){

  //get angular velocity read by encoder (bare motor output)
  double radPerSec_motor = this->getMotorRadPerSec();

  //get angular velocity of output shaft (post-gearbox output) for appropriate motor type
  double radPerSec_output;
  if (this->_isDriveMotor)
    radPerSec_output = radPerSec_motor / double(this->_driveMotor.getGearRatio());
  else
    radPerSec_output = radPerSec_motor / double(this->_componentMotor.getGearRatio());

  //return calculated angular velocity of motor output shaft in radians/second
  return radPerSec_output;
}

//return motor output shaft speed in revolutions per minute
//this value is calculated taking into account any reduction gearing
double EncoderMotor::getOutputRPM()
{

  //get angular velocity read by encoder (bare motor output)
  double RPM_motor = this->getMotorRPM();

  //get angular velocity of output shaft (post-gearbox output) for appropriate motor type
  double RPM_output;;
  if (this->_isDriveMotor)
    RPM_output = RPM_motor / double(this->_driveMotor.getGearRatio());
  else
    RPM_output = RPM_motor / double(this->_componentMotor.getGearRatio());

  //return calculated angular velocity of motor output shaft in revolutions/minute
  return RPM_output;
}
