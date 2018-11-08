//ball (infrared) sensor feedback node
#include <ball_sensor.hpp>
#include <ros/ros.h>
#include <sd_msgs/Ball.h>


int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting ball_sensor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "ball_sensor_node");
  ros::NodeHandle node_private("~");

  //retrieve output pin from parameter server
  int output_pin;
  if (!node_private.getParam("/sensor/ball_sensor/output_pin", output_pin))
  {
    ROS_ERROR("ball sensor output pin not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/sensor/ball_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("ball sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create BallSensor type object using defined outpin pin from parameter server
  BallSensor sensor(output_pin);

  //create sd_msgs/Ball type message to publish ball (proximity) sensor data
  sd_msgs::Ball ball_msg;

  //set ball sensor frame id
  ball_msg.header.frame_id = "ball_sensor_link";

  //create publisher to publish ball sensor message with buffer size 10, and latch set to false
  ros::Publisher ball_sensor_pub = node_private.advertise<sd_msgs::Ball>("ball_sensor", 10, false);

  //set refresh rate of ROS loop to defined refresh rate from parameter server
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current ball sensor reading
    ball_msg.header.stamp = ros::Time::now();

    //check whether a ball is currently detected by sensor and set message data
    ball_msg.ball_detected = sensor.ballDetected();

    //publish ball sensor message
    ball_sensor_pub.publish(ball_msg);

    //spin once because ROS
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
