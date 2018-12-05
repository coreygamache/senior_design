#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/DriveMotors.h>
#include <signal.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

//global variables
bool autonomous_control = false;
unsigned char dirValues[2] = {0, 0}; //motor direction values (0 = forward, 1 = reverse), {left direction, right direction}
unsigned char pwmValues[2] = {0, 0}; //motor pwm output values, {left PWM value, right PWM value}

//pin variables
//must be global so that they can be accessed by callback function
int left_motor_led_pin;
int right_motor_led_pin;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //set all pins LOW
  digitalWrite(left_motor_led_pin, LOW);
  digitalWrite(right_motor_led_pin, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on motor_(num) topic
void controlCallback(const sd_msgs::Control::ConstPtr& msg)
{

  //if autonomous control is different than received value, set to received value
  if (autonomous_control != msg->autonomous_control)
  {

    //set value to received value
    autonomous_control = msg->autonomous_control;

    //if autonomous control was just enabled then reset direction and pwm values for safety
    if (autonomous_control)
    {
      dirValues[0] = 0;
      dirValues[1] = 0;
      pwmValues[0] = 0;
      pwmValues[1] = 0;
    }

  }

}

//callback function called to process messages on drive_motors topic
void driveMotorsCallback(const sd_msgs::DriveMotors::ConstPtr& msg)
{

  //only process drive motor messages if autonomous control is deactivated
  if (!autonomous_control)
  {

    //check left motor direction and change if necessary
    if (dirValues[0] != msg->left_motor_dir)
    {

      //if requested left motor direction is a valid value, change direction
      if ((msg->left_motor_dir == 0) || (msg->left_motor_dir == 1))
        dirValues[0] = msg->left_motor_dir;

    }

    //check right motor direction and change if necessary
    if (dirValues[1] != msg->right_motor_dir)
    {

      //if requested right motor direction is a valid value, change direction
      if ((msg->right_motor_dir == 0) || (msg->right_motor_dir == 1))
        dirValues[0] = msg->right_motor_dir;

    }

    //verify left motor PWM value is within PWM limits
    if (msg->left_motor_pwm > 255)
      pwmValues[0] = 255;
    else if (msg->left_motor_pwm < 0)
      pwmValues[0] = 0;
    else
      pwmValues[0] = msg->left_motor_pwm;

    //verify right motor PWM value is within PWM limits
    if (msg->right_motor_pwm > 255)
      pwmValues[1] = 255;
    else if (msg->right_motor_pwm < 0)
      pwmValues[1] = 0;
    else
      pwmValues[1] = msg->right_motor_pwm;

  }

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting drive_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "drive_motor_node");
  ros::NodeHandle node_private("~");

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve arduino i2c address from parameter server (global parameters)
  int i2c_address;
  if (!node_private.getParam("/arduino/i2c_address", i2c_address))
  {
    ROS_ERROR("[drive_motor_node] arduino i2c address not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve left motor LED pin from parameter server
  if (!node_private.getParam("/hardware/drive_motor/left_motor_led_pin", left_motor_led_pin))
  {
    ROS_ERROR("[drive_motor_node] left drive motor LED pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve right motor LED pin from parameter server
  if (!node_private.getParam("/hardware/drive_motor/right_motor_led_pin", right_motor_led_pin))
  {
    ROS_ERROR("[drive_motor_node] right drive motor LED pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/drive_motor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[drive_motor_node] drive motor node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //initialize i2c protocol and verify connection
  int fd = wiringPiI2CSetup(i2c_address);
  int result; //variable for holding i2c read/write result

  //output notification message and error if one occurs
  if (fd == -1)
    ROS_INFO("[drive_motor_node] error establishing i2c connection: %d", errno);
  else
    ROS_INFO("[drive_motor_node] i2c connection result: %d", fd);

  //create sunscriber to subscribe to control messages message topic with queue size set to 1000
  ros::Subscriber control_sub = node_private.subscribe("control", 1000, controlCallback);

  //create sunscriber to subscribe to drive motor messages message topic with queue size set to 1000
  ros::Subscriber drive_motor_sub = node_private.subscribe("drive_motors", 1000, driveMotorsCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(left_motor_led_pin, OUTPUT);
  pinMode(right_motor_led_pin, OUTPUT);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if autonomous control is disabled then output drive motor manual control values
    if (!autonomous_control)
    {

      //set output values to current direction and pwm values
      unsigned char outputValues[4] = { dirValues[0], pwmValues[0], dirValues[1], pwmValues[1] };

      //output motor PWM values to arduino via i2c protocol
      result = write(fd, outputValues, 4);

      //output notification message if error occurs
      if (result == -1)
      {
        ROS_INFO("[drive_motor_node] error writing to arduino via i2c: %d", errno);
      }
      else
      {

        if (pwmValues[0] > 0)
          digitalWrite(left_motor_led_pin, HIGH);
        else
          digitalWrite(left_motor_led_pin, LOW);

        if(pwmValues[1] > 0)
          digitalWrite(right_motor_led_pin, HIGH);
          digitalWrite(right_motor_led_pin, LOW);

      }

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
