//firing motor control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Mosfet.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int pwmValue = 0; //motor pwm output value

//callback function called to process messages on drive_motors topic
void firingMotorCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //set local enable variable to match value received in message
  if (enable != msg->enable)
    enable = msg->enable;

  //verify motor PWM value is within PWM limits
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
  ROS_INFO("[NODE LAUNCH]: starting firing_wheel_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "firing_wheel_motor_node");
  ros::NodeHandle node_private("~");

  //retrieve mosfet output pin from parameter server
  int output_pin;
  if (!node_private.getParam("/hardware/firing_wheel/output_pin", output_pin))
  {
    ROS_ERROR("firing wheel motor output pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of motor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/firing_wheel/refresh_rate", refresh_rate))
  {
    ROS_ERROR("firing wheel motor node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to firing wheel motor messages message topic with queue size set to 1000
  ros::Subscriber firing_motor_sub = node_private.subscribe("firing_motor", 1000, firingMotorCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if motor is enabled, output requested PWM value to mosfet
    if (enable)
      analogWrite(output_pin, pwmValue);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
