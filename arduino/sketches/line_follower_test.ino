#include "EncoderMotor.h"
#include "PID_v1.h"
#include "QTRSensors.h"

//QTR line sensor pins and constants
const int qtrCalibrationTime = 200;
const unsigned char qtrEmitterPin = 255;
const unsigned char qtrNumSamplesPerSensor = 4;
const unsigned char qtrNumSensors = 6;

//other pin definitions
const int buttonPin = 14;

//QTR line sensor object and variables
const int qtrCenterPosition = ((qtrNumSensors - 1) * 1000) / 2;
unsigned int qtrSensorValues[qtrNumSensors] = {0};
QTRDimmableAnalog qtrSensors((unsigned char[]) {0, 1, 2, 3, 4, 5}, qtrNumSensors, qtrNumSamplesPerSensor, qtrEmitterPin);

//PID setting constants
const double lineFollow_pidKP = 0.5; //proportional value; default 2; wiki: 0.5
const double lineFollow_pidKI = 1.0; //integral value; default 5
const double lineFollow_pidKD = 0.02; //derivative value; default 1
const int lineFollow_pidSampleTime = 200; //ms

//PID variables and objects
double input_lineFollow, output_lineFollow, setPoint_lineFollow = qtrCenterPosition;
PID PID_lineFollow(&input_lineFollow, &output_lineFollow, &setPoint_lineFollow, lineFollow_pidKP, lineFollow_pidKI, lineFollow_pidKD, DIRECT);

//motor variables and objects
const int baseMotorSpeed = 100;
//int dir, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev
EncoderMotor leftEncMotor(255, 255, 255, 1.0, 1, 255, 255, 1.0); //drive motor constructor
EncoderMotor rightEncMotor(255, 255, 255, 1.0, 1, 255, 255, 1.0); //drive motor constructor


//other variables
bool lineFollowComplete = false;

void setup() {

  //set pin modes
  pinMode(buttonPin, INPUT);

  //initialize PID control
  PID_lineFollow.SetMode(AUTOMATIC);
  PID_lineFollow.SetSampleTime(lineFollow_pidSampleTime);

  //initialize serial communication and notify of calibration start
  Serial.flush();
  Serial.begin(9600);
  Serial.println("initializing program");
  Serial.println("press button to begin calibration");

  //wait for user button input before proceeding with calibration
  while (!digitalRead(buttonPin)) {}

  for (int i = 0; i < qtrCalibrationTime; i++) {
    //-->incrementally rotate robot by some amount
    qtrSensors.calibrate();
  }

  //-->return robot to 'forward' facing position

  //notify of calibration completion and line following start
  Serial.println("calibration complete");
  Serial.println("press button to begin line following");

  //wait for user button input before proceeding with line following
  while (!digitalRead(buttonPin)) {}

}

void loop() {

  //if robot is not at end of line then continue to follow line until it is
  while (!lineFollowComplete) {
    lineFollowComplete = lineFollow();

    //check if line following is complete and notify RPi and user if true
    //check is performed in loop so it will only be called once
    if (lineFollowComplete) {
      Serial.println("line following complete");
      //-->output message to raspberry pi
    }

  }

  //if line follow is complete then wait indefinitely
  if (lineFollowComplete) {
    while (1) {
      delay(10000);
    }
  }

}

bool lineFollow() {

  //get current position of line relative to robot
  input_lineFollow = qtrSensors.readLine(qtrSensorValues);

  //if all sensors see black then finish line has been reached; return true
  if ((qtrSensorValues[0] > 750) && (qtrSensorValues[1] > 750) && (qtrSensorValues[2] > 750) && (qtrSensorValues[3] > 750) && (qtrSensorValues[4] > 750) && (qtrSensorValues[5] > 750)) {

    //stop drive motors
    leftEncMotor.stop();
    rightEncMotor.stop()

    //-->return robot to 'forward' facing position if necessary

    //return true to indicate line following is complete
    return true;
  }

  //update PID controller
  bool newOutput = PID_lineFollow.Compute();

  //motor output signals have changed; write to motors
  if (newOutput) {
    int speedModifier = output_lineFollow - qtrCenterPosition;



    //ensure value is valid
  }

}
