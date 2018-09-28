/** blink_leds.cpp
  *
  * this program is a way to test ROS to ROS communication
  * publishes messages from ground station to robot
  * robot receives messages, then blinks LEDs on arduino in accordance
  * comm path: ground station ROS -> robot ROS -> arduino via serial
  *
**/

#include <ros/ros.h>
#include <sstream>
#include <wiringPiI2C.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting drive_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "drive_motor_node");
  ros::NodeHandle node_private("~");

  ros::Rate loop_rate(1);

  int fd;
  int result;

  //initialize i2c protocol
  fd = wiringPiI2CSetup(0x40);
  ROS_INFO("i2c connection result: %d", fd);

  while (ros::ok())
  {

    //output message via i2c protocol
    result = wiringPiI2CWrite(fd, 5);

    //output notification message if error occurs
    if (result == -1)
    {
      ROS_INFO("error");
    }

    //sleep until next cycle
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
