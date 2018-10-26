//line (infrared) sensor feedback node

//this will request line sensor reading from arduino via i2c

#include <errno.h>
#include <ros/ros.h>
#include <sstream>
#include <wiringPiI2C.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting i2c_comm_test_node");

  //initialize node and create node handler
  ros::init(argc, argv, "i2c_comm_test_node");
  ros::NodeHandle node_private("~");

  //set loop rate in Hz
  ros::Rate loop_rate(50);

  //initialize i2c communication and data values
  int fd;
  int result;
  unsigned char data[2] = {0, 0};

  //initialize i2c protocol and verify connection
  fd = wiringPiI2CSetup(0x04);
  ROS_INFO("i2c connection result: %d", fd);

  while (ros::ok())
  {

    //update data values then check if they are above PWM limit
    for (int i = 0; i < 2; i++)
    {

      //iterate data values
      if (i == 0)
      {
        data[i]++;
      }
      else
      {
        data[i]--;
      }

      //verify data values are within PWM limits
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

    //output motor PWM values via i2c protocol
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
