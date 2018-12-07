//gate solenoid control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Mosfet.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int pwmValue = 0; //solenoid pwm output value

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

//callback function called to process messages on gate_solenoid topic
void gateSolenoidCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

    //this conditional is changed to physically turn on/off the motor since PWM output is not possible (see main loop)
    if (enable != msg->enable)
    {

      //set local enable variable to match value received in message
      enable = msg->enable;

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
  ROS_INFO("[NODE LAUNCH]: starting gate_solenoid_node");

  //initialize node and create node handler
  ros::init(argc, argv, "gate_solenoid_node");
  ros::NodeHandle node_private("~");

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve mosfet output pin from parameter server
  if (!node_private.getParam("/hardware/gate_solenoid/output_pin", output_pin))
  {
    ROS_ERROR("[gate_solenoid_node] gate solenoid output pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of solenoid in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/gate_solenoid/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[gate_solenoid_node] gate solenoid node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to gate solenoid messages message topic with queue size set to 1000
  ros::Subscriber gate_solenoid_sub = node_private.subscribe("gate_solenoid", 1000, gateSolenoidCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(output_pin, OUTPUT);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if solenoid has been enabled since last iteration then release a ball:
    //output HIGH to solenoid briefly, then disable again until next request
    if (enable)
    {

      //charge solenoid
      digitalWrite(output_pin, HIGH);

      //wait sufficient time for ball to be released to firing wheel
      delay(500);

      //discharge solenoid until next request
      digitalWrite(output_pin, LOW);
      
    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
