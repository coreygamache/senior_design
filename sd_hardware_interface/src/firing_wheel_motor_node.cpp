//firing motor control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Mosfet.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int pwmValue = 0; //motor pwm output value

//pin variables
//must be global so that they can be accessed by callback function
int output_pin;

//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //set all pins LOW
  digitalWrite(output_pin, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on firing motor topic
void firingMotorCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //this conditional is changed to physically turn on/off the motor since PWM output is not possible (see main loop)
  if (enable != msg->enable)
  {

    //set local enable variable to match value received in message
    enable = msg->enable;

    //set pin to specified value of enable
    digitalWrite(output_pin, enable);

  }

  //check motor pwm value and change if necessary
  if (msg->pwm != pwmValue)
  {

    //verify motor PWM value is within PWM limits
    if (msg->pwm > 255)
      pwmValue = 255;
    else if (msg->pwm < 0)
      pwmValue = 0;
    else
      pwmValue = msg->pwm;

  }

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting firing_wheel_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "firing_wheel_motor_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve mosfet output pin from parameter server
  if (!node_private.getParam("/hardware/firing_wheel/output_pin", output_pin))
  {
    ROS_ERROR("[firing_wheel_motor_node] firing wheel motor output pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of motor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/firing_wheel/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[firing_wheel_motor_node] firing wheel motor node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to firing wheel motor messages message topic with queue size set to 1000
  ros::Subscriber firing_motor_sub = node_public.subscribe("firing_motor", 1000, firingMotorCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(output_pin, OUTPUT);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //the following is removed because PWM output is not possible without running as sudo using wiringPi
    //if motor is enabled, output requested PWM value to motor driver
    //if (enable)
      //analogWrite(output_pin, pwmValue);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
