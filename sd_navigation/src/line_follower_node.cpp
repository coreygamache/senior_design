//line follower logic node
//this will tell arduino to begin line following and report when it is finished

//sends data to arduino to indicate line following to start
//reads arduino on each loop iteration
//arduino sends back 0 until line following is finished, then sends 1 to indicate it's finished

#include <errno.h>
#include <ros/ros.h>
#include <wiringPiI2C.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting line_follower_node");

  //initialize node and create node handler
  ros::init(argc, argv, "line_follower_node");
  ros::NodeHandle node_private("~");

  //retrieve arduino i2c address from parameter server
  int i2c_address;
  if (!node_private.getParam("/arduino/i2c_address", i2c_address))
  {
    ROS_ERROR("arduino i2c address not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/navigation/line_follower_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("line sensor refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //initialize i2c protocol and verify connection
  int fd = wiringPiI2CSetup(0x04);
  int result; //variable for holding i2c read/write result

  //output notification message and error if one occurs
  if (fd == -1)
    ROS_INFO("error establishing i2c connection: %d", errno);
  else
    ROS_INFO("i2c connection result: %d", fd);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  //send message of "1" to arduino to indicate line following should be started
  result = wiringPiI2CWrite(fd, 1);

  //output notification message and error if one occurs
  if (result == -1)
    ROS_INFO("error writing to arduino via i2c: %d", errno);

  while (ros::ok())
  {

    //read current status from arduino to check whether line following is finished
    result = wiringPiI2CRead(fd);

    //output notification message if error occurs
    if (result == -1)
    {
      ROS_INFO("error reading from arduino via i2c:: %d", errno);
    }
    else if (result == 1)
    {
      //indicate line following is complete
    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();

  }

  return 0;
}
