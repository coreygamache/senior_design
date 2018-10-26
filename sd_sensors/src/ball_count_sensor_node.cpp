//ball (infrared) sensor feedback node
#include <ball_sensor.hpp>
#include <ros/ros.h>
#include <sd_sensors/Ball.h>


int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting ball_sensor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "ball_sensor_node");
  ros::NodeHandle node_private("~");

  //get echo pin from parameters
  int output_pin;
  if (!node_private.getParam("ball_sensor/output_pin", output_pin))
  {
    ROS_ERROR("ball sensor output pin not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create Sensor type object using defined echo and trigger pin parameters
  BallSensor sensor(output_pin);

  //create sensor_msgs/Range type message to publish proximity sensor data
  sd_sensors::Ball ball_msg;

  //static message values for HC-SR04 ultrasonic range sensor
  //----------------------------------------------------------

  //set sensor frame id
  ball_msg.header.frame_id = "ball_sensor_link";

  //----------------------------------------------------------

  //create publisher to publish proximity sensor message with buffer size 10, and latch set to false
  ros::Publisher ball_sensor_pub = node_private.advertise<sd_sensors::Ball>("ball_sensor", 10, false);

  //get refresh rate of sensor in hertz
  float refresh_rate;
  if (!node_private.getParam("ball_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("ball sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current distance reading
    ball.header.stamp = ros::Time::now();

    //get distance to nearest object from proximity sensor with 25ms timeout
    ball.ball_detected = sensor.ballDetected();

    //publish proximity sensor range message
    ball_sensor_pub.publish(proximity_msg);

    //spin once because ROS
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}
