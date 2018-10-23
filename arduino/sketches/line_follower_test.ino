#include "EncoderMotor.h"
#include "PID_v1.h"
#include "QTRSensors.h"

//QTR line sensor pins and constants
const int qtrCalibrationTime = 200;
const unsigned char qtrEmitterPin = 255;
const unsigned char qtrNumSamplesPerSensor = 4;
const unsigned char qtrNumSensors = 5; //6;

//other pin definitions
const int buttonPin = 12;

//PID setting constants
const double lineFollow_pidKP = 0.005; //proportional value; default 2; wiki: 0.5
const double lineFollow_pidKI = 0.01; //integral value; default 5
const double lineFollow_pidKD = 0.005; //derivative value; default 1
const int lineFollow_pidSampleTime = 200; //ms

//QTR line sensor object and variables
const int qtrCenterPosition = 2000; //((qtrNumSensors - 1) * 1000) / 2;
unsigned int qtrSensorValues[qtrNumSensors] = {0};
QTRDimmableAnalog qtrSensors((unsigned char[]) {0, 1, 2, 3, 4}, qtrNumSensors, qtrNumSamplesPerSensor, qtrEmitterPin); //one analog pin removed for testing

//motor objects and variables
const double baseMotorSpeed = 75;
const double maxMotorSpeed = 130;
//int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev
EncoderMotor motor_right(4, 6, 5, 255, 250.0, maxMotorSpeed, 2, 255, 12.0);
EncoderMotor motor_left(7, 8, 9, 255, 250.0, maxMotorSpeed, 3, 255, 12.0);

//PID variables and objects
double input_lineFollow, output_lineFollow, setPoint_lineFollow = qtrCenterPosition;
PID PID_lineFollow(&input_lineFollow, &output_lineFollow, &setPoint_lineFollow, lineFollow_pidKP, lineFollow_pidKI, lineFollow_pidKD, DIRECT);

//other variables
int displayCounter = 0;
bool lineFollowComplete = false;

void setup() {

  //set pin modes
  pinMode(buttonPin, INPUT);

  //initialize PID control and set initial set point
  PID_lineFollow.SetMode(AUTOMATIC);
  PID_lineFollow.SetSampleTime(lineFollow_pidSampleTime);
  PID_lineFollow.SetOutputLimits(-1 * maxMotorSpeed, maxMotorSpeed);
  setPoint_lineFollow = 0;

  //initialize serial communication and notify of calibration start
  Serial.flush();
  Serial.begin(9600);
  Serial.println("initializing program");
  Serial.println("press button to begin calibration");

  //wait for user button input before proceeding with calibration
  while (!digitalRead(buttonPin)) {}
  Serial.print("calibrating reflectance sensors");

  for (int i = 0; i < qtrCalibrationTime; i++) {
    //-->incrementally rotate robot by some amount
    qtrSensors.calibrate();
    Serial.print(".");
  }

  //-->return robot to 'forward' facing position

  //notify of calibration completion and line following start
  Serial.println();
  Serial.println("calibration complete");
  Serial.println("press button to begin line following");

  //wait for user button input before proceeding with line following
  while (!digitalRead(buttonPin)) {}
  Serial.println("starting line following program");

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

    //delay(50);

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
  input_lineFollow = qtrSensors.readLine(qtrSensorValues) - qtrCenterPosition;

  //if all sensors see black then finish line has been reached; return true
  /*if ((qtrSensorValues[0] > 750) && (qtrSensorValues[1] > 750) && (qtrSensorValues[2] > 750) && (qtrSensorValues[3] > 750) && (qtrSensorValues[4] > 750) && (qtrSensorValues[5] > 750)) {

    //stop drive motors
    motor_right.stop();
    motor_left.stop();

    //-->return robot to 'forward' facing position if necessary

    //return true to indicate line following is complete
    return true;
  }*/

  //update PID controller
  bool newOutput = PID_lineFollow.Compute();

  //motor output signals have changed; write to motors
  if (newOutput) {

    //calculate new speeds from error: difference in PID output and center position
    double rightMotorSpeed = baseMotorSpeed + output_lineFollow;
    double leftMotorSpeed = baseMotorSpeed - output_lineFollow;

    //verify new right motor speed is valid and output to motor
    if (rightMotorSpeed > motor_right.getComponentMotor().getMaxRPM())
      motor_right.forward();
    else if (rightMotorSpeed < 0)
      motor_right.stop();
    else
      motor_right.forward(rightMotorSpeed);

    //verify new left motor speed is valid and output to motor
    if (leftMotorSpeed > motor_left.getComponentMotor().getMaxRPM())
      motor_left.forward();
    else if (leftMotorSpeed < 0)
      motor_left.stop();
    else
      motor_left.forward(leftMotorSpeed);

    if (displayCounter = 49) {
      Serial.print("PID input: ");
      Serial.println(input_lineFollow);
      Serial.print("PID output: ");
      Serial.println(output_lineFollow);
      Serial.print("motor speeds (left, right): ");
      Serial.print(leftMotorSpeed);
      Serial.print(", ");
      Serial.println(rightMotorSpeed);
      Serial.println();
      displayCounter = 0;
    }
    else {
      displayCounter++;
    }

  }

  //return false to indicate line following is not yet complete
  return false;

}
