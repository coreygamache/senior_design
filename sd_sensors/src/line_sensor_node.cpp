#include <errno.h>
#include <ros/ros.h>
#include <wiringPiI2C.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting line_sensor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "line_sensor_node");
  ros::NodeHandle node_private("~");

  //retrieve arduino i2c address from parameter server
  int i2c_address;
  if (!node_private.getParam("/arduino/i2c_address", i2c_address))
  {
    ROS_ERROR("arduino i2c address not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  int num_sensors;
  if (!node_private.getParam("/sensor/line_sensor/num_sensors", num_sensors))
  {
    ROS_ERROR("number of line sensors not defined in config file: sd_navigation/config/navigation.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/sensor/line_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("line sensor refresh rate not defined in config file: sd_navigation/config/navigation.yaml");
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

  //initialize variable for holding line sensor data
  unsigned char sensor_data[num_sensors] = { };

  //initialize line sensor message
  sd_msgs::LineSensor line_sensor_msg;
  line_sensor_msg.header.frame_id = "0";

  //create publisher to publish line sensor status with buffer size 10, and latch set to false
  ros::Publisher line_sensor_pub = node_private.advertise<sd_msgs::LineSensor>("line_sensors", 10, false);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current iteration
    line_sensor_msg.header.stamp = ros::Time::now();

    //output motor PWM values via i2c protocol
    result = read(fd, sensor_data, 6);

    //output notification message and error if one occurs
    if (result == -1)
      ROS_INFO("error reading from arduino via i2c: %d", errno);

    //set message sensor data to match data received from arduino
    for (int i = 0; i < num_sensors; i++)
    {
      line_sensor_msg.sensor_data[i] = sensor_data[i];
    }

    //publish following following status message
    line_sensor_pub.publish(line_sensor_msg);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
