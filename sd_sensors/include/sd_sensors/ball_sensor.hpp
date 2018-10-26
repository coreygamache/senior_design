#ifndef BALL_SENSOR_HPP
#define BALL_SENSOR_HPP

//create BallSensor class
class BallSensor
{

  //private properties
  private:
    int _outputPin;

  //public properties
  public:

    //default constructor
    BallSensor(int output);

    //basic functions
    int getOutputPin();
    void setOutputPin(int output);

    //advanced functions
    bool ballDetected();

};

#endif
