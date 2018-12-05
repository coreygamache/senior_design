//roller motor control node
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/ComponentMotor.h>
#include <wiringPi.h>

//global variables
bool enable = false;
unsigned int dirValue = 0; //motor direction value (0 = reverse, 1 = forward)
unsigned int pwmValue = 0; //motor pwm output value

//direction pin variables
//must be global so that they can be accessed by callback function
int dir_a_pin;
int dir_b_pin;
int pwm_pin;
int standby_pin;


//callback function called to process messages on roller motor topic
void rollerMotorCallback(const sd_msgs::ComponentMotor::ConstPtr& msg)
{

  //this conditional is changed to physically turn on/off the motor since PWM output is not possible (see main loop)
  if (enable != msg->enable)
  {

    //set local enable variable to match value received in message
    enable = msg->enable;

    //set pin to specified value of enable
    digitalWrite(pwm_pin, enable);

  }


  //check motor direction and change if necessary
  if (dirValue != msg->direction)
  {

    //if requested motor direction is a valid value, change direction
    if ((msg->direction == 0) || (msg->direction == 1))
    {

      //set local direction value to match receive value
      dirValue = msg->direction;

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
  ROS_INFO("[NODE LAUNCH]: starting roller_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "roller_motor_node");
  ros::NodeHandle node_private("~");

  //retrieve motor direction A pin from parameter server
  if (!node_private.getParam("/hardware/roller/dir_a_pin", dir_a_pin))
  {
    ROS_ERROR("[roller_motor_node] roller motor direction A pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor direction B pin from parameter server
  if (!node_private.getParam("/hardware/roller/dir_b_pin", dir_b_pin))
  {
    ROS_ERROR("[roller_motor_node] roller motor direction B pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor PWM pin from parameter server
  if (!node_private.getParam("/hardware/roller/pwm_pin", pwm_pin))
  {
    ROS_ERROR("[roller_motor_node] roller motor PWM pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of motor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/roller/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[roller_motor_node] roller motor node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve motor standy pin from parameter server
  if (!node_private.getParam("/hardware/roller/standby_pin", standby_pin))
  {
    ROS_ERROR("[roller_motor_node] roller motor standby pin not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to roller motor messages message topic with queue size set to 1000
  ros::Subscriber roller_motor_sub = node_private.subscribe("roller_motor", 1000, rollerMotorCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(dir_a_pin, OUTPUT);
  pinMode(dir_b_pin, OUTPUT);
  //pinMode(pwm_pin, PWM_OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  pinMode(standby_pin, OUTPUT);

  //take motor driver out of standby mode if it's currently enabled
  if (!digitalRead(standby_pin))
    digitalWrite(standby_pin, HIGH);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //the following is removed because PWM output is not possible without running as sudo using wiringPi
    //if motor is enabled, output requested PWM value to motor driver
    //if (enable)
      //analogWrite(pwm_pin, pwmValue);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
