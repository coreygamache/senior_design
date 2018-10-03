#include <errno.h>
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

  //set loop rate in Hz
  ros::Rate loop_rate(50);

  int fd;
  int result;
  unsigned char data[2] = {0, 0};

  //initialize i2c protocol
  fd = wiringPiI2CSetup(0x04);
  ROS_INFO("i2c connection result: %d", fd);

  while (ros::ok())
  {

    //update data values then check if they are above PWM limit
    for (int i = 0; i < 2; i++)
    {

       if (i == 0)
       {
       	 data[i]++;
       }
       else
       {
	 data[i]--;
       }

       if (data[i] > 255)
       {
	 data[i] = 0;
       }
       else if (data[i] < 0)
       {
         data[i] = 255;
       }
    }

    //notify of message to be sent
    ROS_INFO("sending message: %d %d", data[0], data[1]);

    //output motor one PWM value  via i2c protocol
    //result = wiringPiI2cWrite(fd, data);
    result = write(fd, data, 2);

    //output notification message if error occurs
    if (result == -1)
    {
      ROS_INFO("error: %d", errno);
    }

    //sleep until next cycle
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
