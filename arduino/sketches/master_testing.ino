//includes
#include <EncoderMotor.h>
#include <PID_v1.h>
#include <QTRSensors.h>
#include <Wire.h>

//define encoder pin constants
const int encoder_left_chanA = 2;
const int encoder_left_chanB = 255;
const int encoder_right_chanA = 3;
const int encoder_right_chanB = 255;

//define motor pin constants
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


//motor parameter constants
const float encoder_counts_per_rev = 48.0;    //motor encoder event counts per revolution
const float motor_gear_ratio = 9.68;          //drive motor gear ratios
const int motor_base_speed = 150;             //default target drive motor speed [pwm value]
const int motor_max_RPM = 990;                //drive motor maximum mechanical RPM [rpm]
const int motor_max_speed = 255;              //imposed speed limit on drive motors [pwm value]


//motor objects and variables
//EncoderMotor(int dir, int pwm, int sleep, float gearRatio, int maxRPM, int channelA, int channelB, float countableEventsPerRev);
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
//line following PID constants
const double line_follow_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double line_follow_pidKI = 0.000; //integral value; default 5
const double line_follow_pidKD = 0.025; //derivative value; default 1
const int line_follow_pidSampleTime = 100; //ms

//left drive motor PID constants
const double motor_left_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_left_pidKI = 0.000; //integral value; default 5
const double motor_left_pidKD = 0.025; //derivative value; default 1
const int motor_left_pidSampleTime = 100; //ms

//right drive motor PID constants
const double motor_right_pidKP = 0.025; //proportional value; default 2; wiki: 0.5
const double motor_right_pidKI = 0.000; //integral value; default 5
const double motor_right_pidKD = 0.025; //derivative value; default 1
const int motor_right_pidSampleTime = 100; //ms

//other constants
const int control_check_iterations = 10;
const int manual_output_iterations = 10;

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
bool autonomous_control = false;
bool line_following_complete = false;
int display_counter = 0;
int loop_counter = 0;


void setup()
{

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

  //PID initial setpoints
  setPoint_line_follow = qtrCenterPosition;
  setPoint_motor_left = 50;
  setPoint_motor_right = 50;

  //initialize i2c communication with slave address 0x04
  Wire.begin(0x04);
  Wire.onReceive(receiveData);

  //attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder_left_chanA), encoder_left_event, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_right_chanA), encoder_right_event, RISING);

  //initialize serial communication and notify of initialization complete
  Serial.begin(9600);

  //wait for successful serial connection
  /*while (!Serial) {
    delay(100);
  }*/
  delay(1000);

  Serial.println("Program initialization complete, beginning control routine.");
  Serial.println("");

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
  Serial.println();
  Serial.println("calibration complete");
  Serial.println("press button to begin line following");

  //flash LED to indicate calibration complete
  for (int i = 0; i < 3; i++) {
    digitalWrite(indicator_led_pin, HIGH);
    delay(500);
    digitalWrite(indicator_led_pin, LOW);
    delay(500);
  }

  //----------------------------------------------------------------------------
  //                      END OF CALIBRATION ROUTINE
  //----------------------------------------------------------------------------

  //wait for user button input before proceeding with line following
  while (!digitalRead(button_pin)) {}
  Serial.println("starting line following program");

  //check initial state of control mode pin
  autonomous_control = digitalRead(control_mode_pin);

}

void loop()
{

  //----------------------------------------------------------------------------
  //                      START OF CONTROL ROUTINE
  //----------------------------------------------------------------------------

  //check current control mode (modePin HIGH indicates autonomous control, LOW indicates manual control)
  //control mode pin is only checked once every control_check_iterations loop iterations to improve PID performance since digital reading is very slow
  if (loop_counter == (control_check_iterations - 1)) {
    Serial.println("reading control mode");
    autonomous_control = digitalRead(control_mode_pin);
    loop_counter = 0;
  }
  else
    loop_counter++;


  //autonomous control loop
  if (autonomous_control) {

    //if robot is not at end of line then continue to follow line until it is
    if (!line_following_complete) {

      //execute line following function to check if complete
      line_following_complete = lineFollow();

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
  //manual mode loop
  //only executed once every manual_output_iterations loop iterations to improve stability
  else if (loop_counter == (manual_output_iterations - 1)) {

    //output status to Serial monitor
    Serial.println("manual mode: outputting new pwm signals to drive motors");

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

    if (display_counter = 49) {
      Serial.print("PID input: ");
      Serial.println(input_line_follow);
      Serial.print("PID output: ");
      Serial.println(output_line_follow);
      Serial.print("motor speeds (left, right): ");
      Serial.print(leftMotorSpeed);
      Serial.print(", ");
      Serial.println(rightMotorSpeed);
      Serial.println();
      display_counter = 0;
    }
    else {
      display_counter++;
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
  if (Wire.available() && byteCount == 4) {

    //initialize variables
    int data;
    //int maxRPM;
    int setPoint;

    for (int i = 0; i < 4; i++) {

      //read data from i2c buffer
      data = Wire.read();

      //get maximum RPM value of respective motor
      /*if (i == 1)
        maxRPM = motor_left.getDriveMotor().getMaxRPM();
      else if (i == 3)
        maxRPM = motor_right.getDriveMotor().getMaxRPM();*/

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
