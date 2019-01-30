//line follower logic node
//this will tell arduino to begin line following and report when it is finished

//sends data to arduino to indicate line following to start
//reads arduino on each loop iteration
//arduino sends back 0 until line following is finished, then sends 1 to indicate it's finished

#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/DisableLineFollowing.h>
#include <sd_msgs/LineFollowing.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool autonomous_control = false;
bool line_following_complete = false; //line following completion status
bool mode_change_requested = false;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const sd_msgs::Control::ConstPtr& msg)
{

  //verify that local mode matches global mode
  if (autonomous_control != msg->autonomous_control)
  {

    //modes do not match; send notification and shut down node
    ROS_INFO("[line_follower_node] local control mode does not match global control mode; killing program");
    ROS_BREAK();

  }

}

//callback function called to process service requests on the disable line following topic
bool DisableLineFollowingCallback(sd_msgs::DisableLineFollowing::Request& req, sd_msgs::DisableLineFollowing::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = true;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
  {

    //change modes
    autonomous_control = !autonomous_control;
    mode_change_requested = true;

    //output notification
    ROS_INFO("[line_follower_node] mode change requested; changing control modes");

  }
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[line_follower_node] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[line_follower_node] mode change requested; indicating node is busy");
  else
    ROS_INFO("[line_follower_node] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting line_follower_node");

  //initialize node and create node handler
  ros::init(argc, argv, "line_follower_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve arduino line following mode pin from parameter server
  int line_following_pin;
  if (!node_private.getParam("/arduino/line_following_pin", line_following_pin))
  {
    ROS_ERROR("[line_follower_node] arduino line following mode pin not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve arduino line following complete from parameter server
  int line_following_complete_pin;
  if (!node_private.getParam("/arduino/line_following_complete_pin", line_following_complete_pin))
  {
    ROS_ERROR("[line_follower_node] arduino line following complete pin not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of line follower node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/navigation/line_follower_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[line_follower_node] line follower node refresh rate not defined in config file: sd_navigation/config/navigation.yaml");
    ROS_BREAK();
  }

  //initialize i2c protocol and verify connection
  wiringPiSetup(); //i2c protocol communication address
  pinMode(line_following_pin, OUTPUT);
  pinMode(line_following_complete_pin, INPUT);

  //create line following message object and set default parameters
  sd_msgs::LineFollowing line_following_msg;
  line_following_msg.header.frame_id = "0";
  line_following_msg.line_following = autonomous_control;
  line_following_msg.complete = line_following_complete;

  //create publisher to publish line following message status with buffer size 10, and latch set to false
  ros::Publisher line_following_pub = node_public.advertise<sd_msgs::LineFollowing>("line_following", 10, true);

  //create service to process service requests on the disable line following topic
  ros::ServiceServer disable_line_following_srv = node_public.advertiseService("disable_line_following", DisableLineFollowingCallback);

  //create sunscriber to subscribe to control messages message topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("/control/control", 1000, controlCallback);

  //publish initial line following status message
  line_following_pub.publish(line_following_msg);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    if (mode_change_requested)
    {

      //set mode change requested to false to prevent mode changing twice for one request
      mode_change_requested = false;

      //set line following complete status to false to force line following start on mode change to autonomous control
      line_following_complete = false;

      //set time and status of current iteration
      line_following_msg.header.stamp = ros::Time::now();
      line_following_msg.line_following = autonomous_control;
      line_following_msg.complete = line_following_complete;

      //publish line following status message
      line_following_pub.publish(line_following_msg);

      //set arduino line following pin status to reflect current control mode
      //if autonomous control is true then pin is set high and vice versa
      if (autonomous_control)
        digitalWrite(line_following_pin, HIGH);
      else
        digitalWrite(line_following_pin, LOW);

    }
    //if autonomous control is enabled then check if line following is complete
    else if (autonomous_control && !line_following_complete && digitalRead(line_following_complete_pin))
    {

      //set line following complete status to true
      line_following_complete = true;

      //set time and status of current iteration
      line_following_msg.header.stamp = ros::Time::now();
      line_following_msg.complete = line_following_complete;

      //publish line following status message
      line_following_pub.publish(line_following_msg);

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();

  }

  return 0;
}
