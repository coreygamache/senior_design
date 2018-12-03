//conveyor motor control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/ComponentMotor.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int dirValue = 0; //motor direction value (0 = reverse, 1 = forward)
unsigned int pwmValue = 0; //motor pwm output value

//callback function called to process messages on drive_motors topic
void conveyorMotorCallback(const sd_msgs::ComponentMotor::ConstPtr& msg)
{

  //set local enable variable to match value received in message
  if (enable != msg->enable)
    enable = msg->enable;

  //check motor direction and change if necessary
  if (dirValues != msg->direction)
  {

    //if requested motor direction is a valid value, change direction
    if ((msg->direction == 0) || (msg->direction == 1))
    {

      //set local direction value to match receive value
      dirValues = msg->direction;

      //change pin output values to achieve requested direction
      if (dirValue == 0)
      {
        digitalWrite(dir_b_pin, LOW);
        digitalWrite(dir_a_pin, HIGH);
      }
      else
      {
        digitalWrite(dir_a_pin, LOW);
        digitalWrite(dir_b_pin, HIGH);
      }


    }

  }

  //verify left motor PWM value is within PWM limits
  if (msg->pwm > 255)
    pwmValues = 255;
  else if (msg->pwm < 0)
    pwmValues = 0;
  else
    pwmValues = msg->pwm;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting conveyor_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "conveyor_motor_node");
  ros::NodeHandle node_private("~");

  //retrieve motor direction A pin from parameter server
  int dir_a_pin;
  if (!node_private.getParam("/hardware/conveyor/dir_a_pin", dir_a_pin))
  {
    ROS_ERROR("conveyor motor direction A pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor direction B pin from parameter server
  int dir_b_pin;
  if (!node_private.getParam("/hardware/conveyor/dir_b_pin", dir_b_pin))
  {
    ROS_ERROR("conveyor motor direction B pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor PWM pin from parameter server
  int pwm_pin;
  if (!node_private.getParam("/hardware/conveyor/pwm_pin", pwm_pin))
  {
    ROS_ERROR("conveyor motor PWM pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of motor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/conveyor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("conveyor motor node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor standy pin from parameter server
  int standby_pin;
  if (!node_private.getParam("/hardware/conveyor/standby_pin", standby_pin))
  {
    ROS_ERROR("conveyor motor standby pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create sunscriber to subscribe to drive motor messages message topic with queue size set to 1000
  ros::Subscriber conveyor_motor_sub = node_private.subscribe("conveyor_motor", 1000, conveyorMotorCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if motor is enabled, output requested PWM value to motor driver
    if (enable)
      analogWrite(pwm_pin, pwmValue);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
