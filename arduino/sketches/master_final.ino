//includes
#include <EncoderMotor.h>
#include <PID_v1.h>
#include <QTRSensors.h>
#include <Wire.h>

//--------------------------------------COMMON TUNING PARAMETERS-------------------------------------

//motor PID speed setting contants
const int motor_base_speed = 60;    //default target drive motor speed for line following [pwm value]
const int motor_min_speed = 40;

//motor non-PID speed setting constants
const int lf_rotate_speed = 40;     //outer wheel speed when rotating
const int lf_inner_speed_one = 30;  //inner wheel speed when one sensor is on line
const int lf_outer_speed_one = 50;  //outer wheel speed when one sensor is on line
const int lf_inner_speed_two = 30;  //inner wheel speed when two sensors are on line
const int lf_outer_speed_two = 75;  //outer wheel speed when two sensors are on line

//sensor value settings
const int inner_sensor_value = 10;
const int outer_sensor_value = 20;

//PID setting constants
//line following PID constants
//BTD:  MBS  = 70, kP = 0.010, kI = 0.000, kD = 0.200, PST = 50
const double line_follow_pidKP = 0.010; //proportional value; default 2; wiki: 0.5
const double line_follow_pidKI = 0.002; //integral value; default 5
const double line_follow_pidKD = 0.200; //derivative value; default 1
const int line_follow_pidSampleTime = 33; //ms

//left drive motor PID constants
const double motor_left_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_left_pidKI = 0.000; //integral value; default 5
const double motor_left_pidKD = 0.000; //derivative value; default 1
const int motor_left_pidSampleTime = 100; //ms

//right drive motor PID constants
const double motor_right_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_right_pidKI = 0.000; //integral value; default 5
const double motor_right_pidKD = 0.025; //derivative value; default 1
const int motor_right_pidSampleTime = 100; //ms

//------------------------------------END COMMON TUNING PARAMETERS-----------------------------------

//define encoder pin constants
const int encoder_left_chanA = 255; //2
const int encoder_left_chanB = 255;
const int encoder_right_chanA = 255; //3
const int encoder_right_chanB = 255;

const int motor_left_dir = 7;
const int motor_left_pwm = 9;
const int motor_left_sleep = 255;
const int motor_right_dir = 8;
const int motor_right_pwm = 10;
const int motor_right_sleep = 255;

//other pin definitions
const int button_pin = 12;                    //input pin for calibration/line following start button
const int control_mode_pin = 11;              //input pin indicating control mode: HIGH = autonomous, LOW = manual
const int indicator_led_pin = 5;
const int line_following_complete_pin = 4;
const int mode_led_pin = 35;

//motor parameter constants
const float encoder_counts_per_rev = 48.0;    //motor encoder event counts per revolution
const float motor_gear_ratio = 34.041;        //drive motor gear ratios
const int motor_max_RPM = 290;                //drive motor maximum mechanical RPM [rpm]

//motor objects and variables
//EncoderMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev);
EncoderMotor motor_left(motor_left_dir, motor_left_pwm, motor_left_sleep, motor_gear_ratio, motor_max_RPM, encoder_left_chanA, encoder_left_chanB, encoder_counts_per_rev);
EncoderMotor motor_right(motor_right_dir, motor_right_pwm, motor_right_sleep, motor_gear_ratio, motor_max_RPM, encoder_right_chanA, encoder_right_chanB, encoder_counts_per_rev);

//QTR line sensor pins and constants
const int qtrCalibrationTime = 300;
const unsigned char qtrNumSamplesPerSensor = 4;
const unsigned char qtrNumSensors = 6;

//QTR line sensor object and variables
const int qtrCenterPosition = 2500; //((qtrNumSensors - 1) * 1000) / 2;
unsigned int qtrSensorValues[qtrNumSensors] = {0};
QTRDimmableAnalog qtrSensors((unsigned char[]) {0, 1, 2, 3, 4, 5}, qtrNumSensors, qtrNumSamplesPerSensor, QTR_NO_EMITTER_PIN);

//other constants
const int control_check_iterations = 5;
const int manual_output_iterations = 5;

//PID variables
double input_line_follow, output_line_follow, setPoint_line_follow = 0;
double input_motor_left, output_motor_left, setPoint_motor_left = 0;
double input_motor_right, output_motor_right, setPoint_motor_right = 0;

//PID objects
PID PID_line_follow(&input_line_follow, &output_line_follow, &setPoint_line_follow, line_follow_pidKP, line_follow_pidKI, line_follow_pidKD, DIRECT);
PID PID_motor_left(&input_motor_left, &output_motor_left, &setPoint_motor_left, motor_left_pidKP, motor_left_pidKI, motor_left_pidKD, DIRECT);
PID PID_motor_right(&input_motor_right, &output_motor_right, &setPoint_motor_right, motor_right_pidKP, motor_right_pidKI, motor_right_pidKD, DIRECT);

//motor direction variables
volatile bool motor_left_direction = 0;
volatile bool motor_right_direction = 0;

//other variables
bool autonomous_control = false;
bool line_following_complete = false;
int display_counter = 0;
int loop_counter = 0;


void setup()
{
  
  //delay for i2c bus to start, initialize serial communication and notify
  Serial.begin(9600);
  Serial.println("initializing i2c bus");
  delay(1000);
  
  
  //initialize i2c communication with slave address 0x04
  Wire.begin(4);
  Wire.onReceive(receiveData);
  Serial.println("i2c bus initialized");

  //set pin modes
  pinMode(button_pin, INPUT);
  pinMode(indicator_led_pin, OUTPUT);
  pinMode(line_following_complete_pin, OUTPUT);
  pinMode(control_mode_pin, INPUT);

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

  //attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder_left_chanA), encoder_left_event, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_right_chanA), encoder_right_event, RISING);


  //----------------------------------------------------------------------------
  //                      START OF CALIBRATION ROUTINE
  //----------------------------------------------------------------------------

  Serial.println("press button to begin calibration");

  //turn on LED to indicate waiting for button press
  digitalWrite(indicator_led_pin, HIGH);

  //wait for user button input before proceeding with calibration
  while (!digitalRead(button_pin)) {}
  Serial.print("calibrating reflectance sensors");

  //turn off LED to indicate calibrating
  digitalWrite(indicator_led_pin, LOW);

  for (int i = 0; i < qtrCalibrationTime; i++) {
    //-->incrementally rotate robot by some amount
    qtrSensors.calibrate();
    Serial.print(".");
  }

  //-->return robot to 'forward' facing position

  //notify of calibration completion and line following start
  Serial.println("\ncalibration complete\npress button to begin line following");

  //flash LED to indicate calibration complete
  for (int i = 0; i < 2; i++) {
    digitalWrite(indicator_led_pin, HIGH);
    delay(250);
    digitalWrite(indicator_led_pin, LOW);
    delay(250);
  }

  //----------------------------------------------------------------------------
  //                      END OF CALIBRATION ROUTINE
  //----------------------------------------------------------------------------

  //turn on line following complete LED to indicate ready to start line following
  digitalWrite(indicator_led_pin, HIGH);  

  //wait for user button input before proceeding with line following
  while (!digitalRead(button_pin)) {}
  Serial.println("starting main control program\n");
  
  //turn off line following complete LED at start of line follow
  digitalWrite(indicator_led_pin, LOW);

  //check initial state of control mode pin
  autonomous_control = digitalRead(control_mode_pin);
  //autonomous_control = true;

}

void loop()
{

  //----------------------------------------------------------------------------
  //                      START OF CONTROL ROUTINE
  //----------------------------------------------------------------------------

  //check current control mode (modePin HIGH indicates autonomous control, LOW indicates manual control)
  //control mode pin is only checked once every control_check_iterations loop iterations to improve PID performance since digital reading is very slow
  if (loop_counter == (control_check_iterations - 1)) {
    autonomous_control = digitalRead(control_mode_pin);
    digitalWrite(mode_led_pin, autonomous_control);
    loop_counter = 0;
  }
  else
    loop_counter++;


  //autonomous mode control loop
  if (autonomous_control) {

    //if robot is not at end of line then continue to follow line until it is
    if (!line_following_complete) {

      //execute line following function to check if complete
      line_following_complete = lineFollowPID();

      //if line following is complete then notify RPi and user if true
      if (line_following_complete) {

        //output status change to serial monitor
        Serial.println("line following complete");

        //output HIGH to line following complete indicator pin
        digitalWrite(line_following_complete_pin, HIGH);

        //wait indefinitely until mode change
        while (digitalRead(control_mode_pin)) {
          delay(100);
        }

        //when mode changes reset line following complete back to false
        line_following_complete = false;

        //set line following complete pin LOW to indicate status to Raspberry Pi
        digitalWrite(line_following_complete_pin, LOW);

      }

    }

  }
  //manual mode control loop
  //only executed once every manual_output_iterations loop iterations to improve stability
  else if (loop_counter == (manual_output_iterations - 1)) {
    
    map(setPoint_motor_right, 0, 255, 0, 200);

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
  
  //check for end of line
    if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //stop drive motors
    motor_left.stop();
    motor_right.stop();

    //return true to indicate line following is complete
    return true;

  }
  
  //only left-most sensor is on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //rotate left
    motor_left.stop();
    motor_right.forward(lf_rotate_speed);

  }
  
  //two left sensors are on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //hard turn left
    motor_left.forward(lf_inner_speed_two);
    motor_right.forward(lf_outer_speed_two);

  }
  
  //only inner-left sensor is on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //soft turn left
    motor_left.forward(lf_inner_speed_one);
    motor_right.forward(lf_outer_speed_one);

  }
  
  //only inner-right sensor is on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //soft turn right
    motor_left.forward(lf_outer_speed_one);
    motor_right.forward(lf_inner_speed_one);

  }
  
  //two right sensors are on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //hard turn right
    motor_left.forward(lf_outer_speed_two);
    motor_right.forward(lf_inner_speed_two);

  }
  
  //only right-most sensor is on line
  if ((qtrSensorValues[0] > 500) && (qtrSensorValues[1] > 500) && (qtrSensorValues[2] > 500) && (qtrSensorValues[3] > 500)) {

    //rotate right
    motor_left.forward(lf_rotate_speed);
    motor_right.stop();

  }
  
  //return false to indicate not finished
  return false;
  
}

bool lineFollowPID() {
  
  //get current position of line relative to robot
  input_line_follow = qtrSensors.readLine(qtrSensorValues, QTR_EMITTERS_ON) - qtrCenterPosition;
  
  //qtrSensors.read(qtrSensorValues);
  //qtrSensors.readCalibrated(qtrSensorValues);
  
  /*Serial.print("qtr (L->R): ");
  Serial.print(qtrSensorValues[0]);
  Serial.print(" ");
  Serial.print(qtrSensorValues[1]);
  Serial.print(" ");
  Serial.print(qtrSensorValues[2]);
  Serial.print(" ");
  Serial.print(qtrSensorValues[3]);
  Serial.print(" ");
  Serial.print(qtrSensorValues[4]);
  Serial.print(" ");
  Serial.println(qtrSensorValues[5]);*/
  
  //Serial.print("input: ");
  //Serial.println(input_line_follow);

  //if all sensors see black then finish line has been reached; return true
  if ((qtrSensorValues[0] >= 500) && (qtrSensorValues[1] >= 500) && (qtrSensorValues[2] >= 500) && (qtrSensorValues[3] >= 500) && (qtrSensorValues[4] >= 500) && (qtrSensorValues[5] >= 500)) {

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

    //Serial.print("error: ");
    //Serial.println(output_line_follow);
  
    //calculate new speeds from error: difference in PID output and center position
    int leftMotorSpeed = motor_base_speed - output_line_follow;
    int rightMotorSpeed = motor_base_speed + output_line_follow;
    
    if (leftMotorSpeed < motor_min_speed)
      leftMotorSpeed = 0;
      
    if (rightMotorSpeed < motor_min_speed)
      rightMotorSpeed = 0;

    //output new speed to motors with correct direction
    motor_left.forward(leftMotorSpeed);
    motor_right.forward(rightMotorSpeed);

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
  if (Wire.available() && byteCount == 4) {

    //initialize variables
    int data;
    //int maxRPM;
    int setPoint;

    for (int i = 0; i < 4; i++) {

      //read data from i2c buffer
      data = Wire.read();

      //if current byte is a PWM value then verify value is within valid range
      if ((i == 1) || (i == 3)) {

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
  //if byte count is wrong then read all available data off buffer to clear it of invalid data
  else {
    while (Wire.available())
      Wire.read();
  }

}