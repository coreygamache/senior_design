#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/DriveMotors.h>
#include <wiringPiI2C.h>

//global variables
unsigned char pwmValues[2] = {0, 0}; //variable for holding motor pwm output values, {left PWM value, right PWM value}

//callback function called to process messages on motor_(num) topic
void driveMotorsCallback(const sd_msgs::DriveMotors::ConstPtr& msg)
{

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

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting drive_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "drive_motor_node");
  ros::NodeHandle node_private("~");

  //retrieve arduino i2c address from parameter server
  int i2c_address;
  if (!node_private.getParam("/arduino/i2c_address", i2c_address))
  {
    ROS_ERROR("arduino i2c address not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/drive_motor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("line sensor refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //initialize i2c protocol and verify connection
  int fd = wiringPiI2CSetup(i2c_address);
  int result; //variable for holding i2c read/write result

  //output notification message and error if one occurs
  if (fd == -1)
    ROS_INFO("error establishing i2c connection: %d", errno);
  else
    ROS_INFO("i2c connection result: %d", fd);

  //create sunscriber to subscribe to drive motor messages message topic with queue size set to 1000
  ros::Subscriber drive_motor_sub = node_private.subscribe("drive_motors", 1000, driveMotorsCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //output motor PWM values to arduino via i2c protocol
    result = write(fd, pwmValues, 2);

    //output notification message if error occurs
    if (result == -1)
      ROS_INFO("error writing to arduino via i2c: %d", errno);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
