//gate solenoid control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Mosfet.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int pwmValue = 0; //solenoid pwm output value

//callback function called to process messages on gate_solenoid topic
void gateSolenoidCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //set local enable variable to match value received in message
  if (enable != msg->enable)
    enable = msg->enable;

  //verify solenoid PWM value is within PWM limits
  if (msg->pwm > 255)
    pwmValue = 255;
  else if (msg->pwm < 0)
    pwmValue = 0;
  else
    pwmValue = msg->pwm;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting gate_solenoid_node");

  //initialize node and create node handler
  ros::init(argc, argv, "gate_solenoid_node");
  ros::NodeHandle node_private("~");

  //retrieve mosfet output pin from parameter server
  int output_pin;
  if (!node_private.getParam("/hardware/gate_solenoid/output_pin", output_pin))
  {
    ROS_ERROR("gate solenoid output pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of solenoid in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/gate_solenoid/refresh_rate", refresh_rate))
  {
    ROS_ERROR("gate solenoid node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to gate solenoid messages message topic with queue size set to 1000
  ros::Subscriber gate_solenoid_sub = node_private.subscribe("gate_solenoid", 1000, gateSolenoidCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if solenoid is enabled, output requested PWM value to mosfet
    if (enable)
      analogWrite(output_pin, pwmValue);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
