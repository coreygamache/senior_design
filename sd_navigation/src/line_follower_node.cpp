//line follower logic node
//this will tell arduino to begin line following and report when it is finished

//sends data to arduino to indicate line following to start
//reads arduino on each loop iteration
//arduino sends back 0 until line following is finished, then sends 1 to indicate it's finished

#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/LineFollowing.h>
#include <wiringPiI2C.h>

//global variables
bool line_following = false; //current line following status
bool line_following_completed = false; //line following completion status
int fd; //i2c protocol communication address

//callback function called to process messages on motor_(num) topic
void controlCallback(const sd_msgs::Control::ConstPtr& msg)
{

  //handle message differently depending on current line following status and message request
  //line following was disabled, now enable it
  if (!line_following && msg->autonomous_control)
  {

    //send message of "1" to arduino to indicate line following should be started
    int result = wiringPiI2CWrite(fd, 1);

    //output notification message and error if one occurs
    //otherwise message was sent; set line following to true
    if (result == -1)
      ROS_INFO("error writing to arduino via i2c: %d", errno);
    else
      line_following = true;

  }
  //line following was enabled, now disable it
  else if (line_following && !msg->autonomous_control)
  {

    //send message of "0" to arduino to indicate line following should be stopped
    int result = wiringPiI2CWrite(fd, 0);

    //output notification message and error if one occurs
    //otherwise message was sent; set line following to false
    if (result == -1)
      ROS_INFO("error writing to arduino via i2c: %d", errno);
    else
      line_following = false;

  }

}

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
    ROS_ERROR("arduino i2c address not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/navigation/line_follower_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("line follower node refresh rate not defined in config file: sd_navigation/config/navigation.yaml");
    ROS_BREAK();
  }

  //initialize i2c protocol and verify connection
  fd = wiringPiI2CSetup(i2c_address);
  int result; //variable for holding i2c read/write result

  //output notification message and error if one occurs
  if (fd == -1)
    ROS_INFO("error establishing i2c connection: %d", errno);
  else
    ROS_INFO("i2c connection result: %d", fd);

  //create line following message object and set default parameters
  sd_msgs::LineFollowing line_following_msg;
  line_following_msg.header.frame_id = "0";
  line_following_msg.line_following = line_following;
  line_following_msg.completed = line_following_completed;

  //create publisher to publish line following message status with buffer size 10, and latch set to false
  ros::Publisher line_following_pub = node_private.advertise<sd_msgs::LineFollowing>("line_following", 10, false);

  //create sunscriber to subscribe to control messages message topic with queue size set to 1000
  ros::Subscriber control_sub = node_private.subscribe("control", 1000, controlCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set time of current iteration
    line_following_msg.header.stamp = ros::Time::now();

    //set line following field of message to current line following status
    line_following_msg.line_following = line_following;

    //perform the following if line following is enabled and not yet complete (line_following must be set to true by a control message)
    if (line_following && !line_following_completed)
    {

      //read current status from arduino to check whether line following is finished
      result = wiringPiI2CRead(fd);

      //check whether line following is finished and output notification message if error occurs
      if (result == -1)
      {
        ROS_INFO("error reading from arduino via i2c:: %d", errno);
      }
      else if (result == 1)
      {

        //indicate line following is complete and change status of message field
        line_following_completed = true;
        line_following_msg.completed = true;

      }

    }

    //publish line following status message
    line_following_pub.publish(line_following_msg);

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();

  }

  return 0;
}
