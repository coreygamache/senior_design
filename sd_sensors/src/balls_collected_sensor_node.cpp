//node for determining number of balls collected using proximity (infrared) sensor
#include <ball_sensor.hpp>
#include <ros/ros.h>
#include <sd_msgs/BallsCollected.h>


int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting balls_collected_sensor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "balls_collected_sensor_node");
  ros::NodeHandle node_private("~");

  //retrieve ball sensor output pin from parameter server
  int output_pin;
  if (!node_private.getParam("/sensor/balls_collected_sensor/output_pin", output_pin))
  {
    ROS_ERROR("balls collected sensor output pin not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of ball sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/sensor/balls_collected_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("balls collected sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create BallSensor type object using defined output pin from parameter server
  BallSensor sensor(output_pin);

  //create sd_msgs/BallsCollected type message to publish number of balls collected
  sd_msgs::BallsCollected balls_collected_msg;

  //set message frame id to 0 to indicate there is no frame
  balls_collected_msg.header.frame_id = "0";

  //create publisher to publish number of balls collected message with buffer size 10, and latch set to false
  ros::Publisher balls_collected_sensor_pub = node_private.advertise<sd_msgs::BallsCollected>("balls_collected", 10, false);

  //set refresh rate of ROS loop to defined refresh rate of sensor parameter from parameter server
  ros::Rate loop_rate(refresh_rate);

  //define logic variables for counting balls
  int balls_collected = 0;
  bool ball_detected = false;
  bool last_reading = false;

  while (ros::ok())
  {

    //set time of current sensor reading
    balls_collected_msg.header.stamp = ros::Time::now();

    //get current sensor reading
    ball_detected = sensor.ballDetected();

    //if a ball is detected now, and a ball was not detected in the last reading then a ball was collected
    //add a ball to total count
    if (ball_detected && !last_reading)
      balls_collected++;

    //set balls collected data in message to number of balls currently collected
    balls_collected_msg.balls_collected = balls_collected;

    //publish number of balls collected message
    balls_collected_sensor_pub.publish(balls_collected_msg);

    //set last_reading variable to match current reading
    last_reading = ball_detected;

    //process callback function calls
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }
  return 0;
}