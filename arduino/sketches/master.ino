//includes
#include <EncoderMotor.h>
#include <PID_v1.h>
#include "QTRSensors.h"
#include <Wire.h>

//define encoder pin constants
const int encoder_left_chanA = 2;
const int encoder_left_chanB = 255;
const int encoder_right_chanA = 3;
const int encoder_right_chanB = 255;

//define motor pin constants
const int motor_left_dir = 7;
const int motor_left_pwm = 9;
const int motor_left_sleep = 8;
const int motor_right_dir = 4;
const int motor_right_pwm = 5;
const int motor_right_sleep = 6;

//other pin definitions
const int buttonPin = 12;
const int modePin = 11;

//motor parameter constants
const float encoder_counts_per_rev = 48.0;    //motor encoder event counts per revolution
const float motor_gear_ratio = 9.68;          //drive motor gear ratios
const int motor_max_speed = 255;              //imposed speed limit on drive motors
const int motor_base_speed = 150;             //default target drive motor speed
const int motor_max_RPM = 990;                //drive motor maximum mechanical RPM

//motor objects and variables
//int inputA, int inputB, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev
EncoderMotor motor_left(motor_left_dir, motor_left_pwm, motor_left_sleep, motor_gear_ratio, motor_max_speed, encoder_left_chanA, encoder_left_chanB, encoder_counts_per_rev);
EncoderMotor motor_right(motor_right_dir, motor_right_pwm, motor_right_sleep, motor_gear_ratio, motor_max_speed, encoder_right_chanA, encoder_right_chanB, encoder_counts_per_rev);

//QTR line sensor pins and constants
const int qtrCalibrationTime = 200;
const unsigned char qtrEmitterPin = 255;
const unsigned char qtrNumSamplesPerSensor = 4;
const unsigned char qtrNumSensors = 6;

//QTR line sensor object and variables
const int qtrCenterPosition = 2500; //((qtrNumSensors - 1) * 1000) / 2;
unsigned int qtrSensorValues[qtrNumSensors] = {0};
QTRDimmableAnalog qtrSensors((unsigned char[]) {0, 1, 2, 3, 4, 5}, qtrNumSensors, qtrNumSamplesPerSensor, qtrEmitterPin);

//PID setting constants
//line following
const double line_follow_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double line_follow_pidKI = 0.000; //integral value; default 5
const double line_follow_pidKD = 0.025; //derivative value; default 1
const int line_follow_pidSampleTime = 100; //ms

//left drive motor
const double motor_left_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_left_pidKI = 0.000; //integral value; default 5
const double motor_left_pidKD = 0.025; //derivative value; default 1
const int motor_left_pidSampleTime = 100; //ms

//right drive motor
const double motor_right_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_right_pidKI = 0.000; //integral value; default 5
const double motor_right_pidKD = 0.025; //derivative value; default 1
const int motor_right_pidSampleTime = 100; //ms

//PID variables
double input_line_follow, output_line_follow, setPoint_line_follow = qtrCenterPosition;
double input_motor_left, output_motor_left, setPoint_motor_left;
double input_motor_right, output_motor_right, setPoint_motor_right;

//PID objects
PID PID_line_follow(&input_line_follow, &output_line_follow, &setPoint_line_follow, line_follow_pidKP, line_follow_pidKI, line_follow_pidKD, DIRECT);
PID PID_motor_left(&input_motor_left, &output_motor_left, &setPoint_motor_left, motor_left_pidKP, motor_left_pidKI, motor_left_pidKD, DIRECT);
PID PID_motor_right(&input_motor_right, &output_motor_right, &setPoint_motor_right, motor_right_pidKP, motor_right_pidKI, motor_right_pidKD, DIRECT);

//motor direction variables
volatile bool motor_left_direction = 0;
volatile bool motor_right_direction = 0;

//other variables
int displayCounter = 0;
bool lineFollowComplete = false;


void setup()
{

  //set pin modes
  pinMode(buttonPin, INPUT);
  pinMode(modePin, INPUT);

  //initialize PID controllers and set initial setpoints
  //PID modes
  PID_line_follow.SetMode(AUTOMATIC);
  PID_motor_left.SetMode(AUTOMATIC);
  PID_motor_right.SetMode(AUTOMATIC);

  //PID output limits
  PID_line_follow.SetOutputLimits(-255, 255);
  PID_motor_left.SetOutputLimits(-255, 255);
  PID_motor_right.SetOutputLimits(-255, 255);

  //PID sample times
  PID_line_follow.SetSampleTime(line_follow_pidSampleTime);
  PID_motor_left.SetSampleTime(motor_left_pidSampleTime);
  PID_motor_right.SetSampleTime(motor_right_pidSampleTime);

  //PID initial setpoints
  setPoint_line_follow = qtrCenterPosition;
  setPoint_motor_left = 0;
  setPoint_motor_right = 0;

  //initialize i2c communication with slave address 0x04
  Wire.begin(0x04);
  Wire.onReceive(receiveData);

  //attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder_left_chanA), encoder_left_event, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_right_chanA), encoder_right_event, RISING);

  //initialize serial communication and notify of initialization complete
  Serial.begin(9600);
  Serial.println("Program initialization complete, beginning control routine.");
  Serial.println("");

  //----------------------------------------------------------------------------
  //                      START OF CALIBRATION ROUTINE
  //----------------------------------------------------------------------------

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

  //----------------------------------------------------------------------------
  //                      END OF CALIBRATION ROUTINE
  //----------------------------------------------------------------------------

  //wait for user button input before proceeding with line following
  while (!digitalRead(buttonPin)) {}
  Serial.println("starting line following program");

}

void loop()
{

  //----------------------------------------------------------------------------
  //                      START OF CONTROL ROUTINE
  //----------------------------------------------------------------------------

  //check current control mode (modePin HIGH indicates autonomous control, LOW indicates manual control)
  //autonomous control loop
  if (digitalRead(modePin))
  {

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
  //manual mode loop
  else
  {

    //output received PWM values to drive motors
    //left drive motor
    if (motor_left_direction == 0)
      motor_left.forward(setPoint_motor_left);
    else
      motor_left.reverse(setPoint_motor_left);

    //right drive motor
    if (motor_right_direction == 0)
      motor_right.forward(setPoint_motor_right);
    else
      motor_right.reverse(setPoint_motor_right);

  }

  //----------------------------------------------------------------------------
  //                       END OF CONTROL ROUTINE
  //----------------------------------------------------------------------------

}

bool lineFollow() {

  //get current position of line relative to robot
  input_line_follow = qtrSensors.readLine(qtrSensorValues) - qtrCenterPosition;

  //if all sensors see black then finish line has been reached; return true
  if ((qtrSensorValues[0] > 750) && (qtrSensorValues[1] > 750) && (qtrSensorValues[2] > 750) && (qtrSensorValues[3] > 750) && (qtrSensorValues[4] > 750) && (qtrSensorValues[5] > 750)) {

    //stop drive motors
    motor_left.stop();
    motor_right.stop();

    //return true to indicate line following is complete
    return true;

  }

  //update PID controller
  bool newOutput = PID_line_follow.Compute();

  //motor output signals have changed; write to motors
  if (newOutput) {

    //calculate new speeds from error: difference in PID output and center position
    int leftMotorSpeed = motor_base_speed - output_line_follow;
    int rightMotorSpeed = motor_base_speed + output_line_follow;

    //verify new left motor speed is valid and output to motor
    if (leftMotorSpeed > motor_max_speed)
      motor_left.forward(motor_max_speed);
    else if (leftMotorSpeed > 0)
      motor_left.forward(leftMotorSpeed);
    else if (leftMotorSpeed < -255)
      motor_left.reverse(motor_max_speed);
    else
      motor_left.reverse(leftMotorSpeed);

    //verify new right motor speed is valid and output to motor
    if (rightMotorSpeed > motor_max_speed)
      motor_right.forward(motor_max_speed);
    else if (rightMotorSpeed > 0)
      motor_right.forward(rightMotorSpeed);
    else if (rightMotorSpeed < -255)
      motor_right.reverse(motor_max_speed);
    else
      motor_right.reverse(rightMotorSpeed);

    if (displayCounter = 49) {
      Serial.print("PID input: ");
      Serial.println(input_line_follow);
      Serial.print("PID output: ");
      Serial.println(output_line_follow);
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

void encoder_left_event() {
  motor_left.addChannelACount();
}

void encoder_right_event() {
  motor_right.addChannelACount();
}

//receive and set requested motor RPM setpoints
void receiveData(int byteCount) {

  //check if data is available and message is of correct size
  if (Wire.available() && byteCount == 4)
  {

    //initialize variables
    int data;
    //int maxRPM;
    int setPoint;

    for (int i = 0; i < 4; i++)
    {

      //read data from i2c buffer
      data = Wire.read();

      //get maximum RPM value of respective motor
      /*if (i == 1)
        maxRPM = motor_left.getDriveMotor().getMaxRPM();
      else if (i == 3)
        maxRPM = motor_right.getDriveMotor().getMaxRPM();*/

      //if current byte is a PWM value then verify value is within valid range
      if ((i == 1) || (i == 3))
      {

        //verify value is within valid range
        if (data >= 0 && data <= motor_max_RPM)
          setPoint = data;
        else if (data < 0)
          setPoint = 0;
        else if (data > motor_max_RPM)
          setPoint = motor_max_RPM;

      }

      //set corresponding variable value to received data value
      if (i == 0)
        motor_left_direction = bool(data);
      else if (i == 1)
        setPoint_motor_left = double(setPoint);
      else if (i == 2)
        motor_right_direction = bool(data);
      else
        setPoint_motor_right = double(setPoint);

    }

  }

}




/* original program for PID testing

//setPoint = map(pidSetPoint, 0, maxRPM, 0, 255);

//compute current iteration PID control values
input_motor1 = motor1.getRPM();
input_motor2 = motor2.getRPM();
setPoint_motor1 = 10;
setPoint_motor2 = 10;
PID_Motor1.Compute();
PID_Motor2.Compute();

//output new PID control output values to motors
motor1.forward(map(int(output_motor1), 0, motor1.getDriveMotor().getMaxRPM(), 0, 255));
motor2.forward(map(int(output_motor2), 0, motor2.getDriveMotor().getMaxRPM(), 0, 255));

//output current iteration PID input values to serial
Serial.print("Motor 1 Input: ");
Serial.println(input_motor1);
Serial.print("Motor 2 Input: ");
Serial.println(input_motor2);

//output current iteration PID output values to serial
Serial.print("Motor 1 Output: ");
Serial.println(output_motor1);
Serial.print("Motor 2 Output: ");
Serial.println(output_motor2);
Serial.println();

//delay for predefined time before next cycle
delay(msPerCycle);

end of original program */
