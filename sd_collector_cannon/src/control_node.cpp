//control mode node
//handles all tasks involved in switching between manual and autonomous control modes
#include <ros/ros.h>
#include <sd_msgs/ComponentMotor.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/ChangeControlMode.h>
#include <sd_msgs/FiringStatus.h>
#include <sd_msgs/LineFollowing.h>
#include <sd_msgs/Mosfet.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool firing_complete = false;
bool line_following_complete = false;
bool mode_change_requested = false;

//global controller variables
std::vector<int> controller_buttons(13, 0);

//pin variables
//must be global so that they can be accessed by callback functions
int component_motor_standby_pin;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //disable component motor driver
  digitalWrite(component_motor_standby_pin, LOW);

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_buttons = msg->buttons;

  //if mode change controller button is pressed then set change mode request true
  if (controller_buttons[10] == 1)
  {

    //set mode change requested to true to indicate request to change modes
    mode_change_requested = true;

    //reset controller button to prevent mode from toggling twice on one button press
    controller_buttons[10] = 0;

  }

}

//callback function called to process messages on firing status topic
void firingStatusCallback(const sd_msgs::FiringStatus::ConstPtr& msg)
{

  //set local value to match message value
  if (firing_complete != msg->complete)
    firing_complete = msg->complete;

}

//callback function called to process messages on line following topic
void lineFollowingCallback(const sd_msgs::LineFollowing::ConstPtr& msg)
{

  //set local value to match message value
  if (line_following_complete != msg->complete)
    line_following_complete = msg->complete;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting control_node");

  //initialize node and create node handler
  ros::init(argc, argv, "control_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve component from parameter server
  if (!node_private.getParam("/component_motor_driver/standby_pin", component_motor_standby_pin))
  {
    ROS_ERROR("[control_node] component motor driver standby pin not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/control_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[control_node] control mode node refresh rate not defined in config file: sd_collector_cannon/config/control.yaml");
    ROS_BREAK();
  }

  //create control message object and set default parameters
  sd_msgs::Control control_msg;
  control_msg.header.frame_id = "0";

  //create conveyor message object and set default parameters
  sd_msgs::ComponentMotor conveyor_msg;
  conveyor_msg.header.frame_id = "0";
  conveyor_msg.enable = false;
  conveyor_msg.direction = 0;
  conveyor_msg.pwm = 0;

  sd_msgs::ChangeControlMode disable_firing_srv;
  disable_firing_srv.request.mode_change_requested = true;

  sd_msgs::ChangeControlMode disable_line_following_srv;
  disable_line_following_srv.request.mode_change_requested = true;

  sd_msgs::ChangeControlMode disable_manual_control_srv;
  disable_manual_control_srv.request.mode_change_requested = true;

  //create firing motor message object and set default parameters
  sd_msgs::Mosfet firing_motor_msg;
  firing_motor_msg.header.frame_id = "0";
  firing_motor_msg.enable = false;
  firing_motor_msg.pwm = 0;

  //create roller motor message object and set default parameters
  sd_msgs::ComponentMotor roller_msg;
  roller_msg.header.frame_id = "0";
  roller_msg.enable = false;
  roller_msg.direction = 0;
  roller_msg.pwm = 0;

  //create publisher to publish control message status with buffer size 10, and latch set to true
  ros::Publisher control_pub = node_public.advertise<sd_msgs::Control>("control", 10, true);

  //create publisher to publish conveyor motor message with buffer size 10, and latch set to true
  ros::Publisher conveyor_pub = node_public.advertise<sd_msgs::ComponentMotor>("conveyor_motor", 10, true);

  //create publisher to publish firing wheel motor message with buffer size 10, and latch set to true
  ros::Publisher firing_motor_pub = node_public.advertise<sd_msgs::Mosfet>("firing_motor", 10, true);

  //create publisher to publish roller motor message with buffer size 10, and latch set to true
  ros::Publisher roller_pub = node_public.advertise<sd_msgs::ComponentMotor>("roller_motor", 10, true);

  //create service client to send service requests on the disable firing topic
  ros::ServiceClient disable_firing_clt = node_public.serviceClient<sd_msgs::ChangeControlMode>("disable_firing");

  //create service client to send service requests on the disable line following topic
  ros::ServiceClient disable_line_following_clt = node_public.serviceClient<sd_msgs::ChangeControlMode>("disable_line_following");

  //create service client to send service requests on the disable manual control topic
  ros::ServiceClient disable_manual_control_clt = node_public.serviceClient<sd_msgs::ChangeControlMode>("disable_manual_control");

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //create subscriber to subscribe to firing status messages topic with queue size set to 1000
  ros::Subscriber firing_status_sub = node_public.subscribe("firing_status", 1000, firingStatusCallback);

  //create subscriber to subscribe to line following messages topic with queue size set to 1000
  ros::Subscriber line_following_sub = node_public.subscribe("/navigation/line_following", 1000, lineFollowingCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(component_motor_standby_pin, OUTPUT);

  //disable component motor driver standby mode on startup
  digitalWrite(component_motor_standby_pin, HIGH);

  //create variable for status of autonomous control
  bool autonomous_control = false;

  //initialize control message to disable autonomous control globally
  control_msg.header.stamp = ros::Time::now();
  control_msg.autonomous_control = false;
  control_msg.complete = false;
  control_msg.firing_stage = false;
  control_msg.navigation_stage = false;

  //publish initial control message to disable autonomous control globally
  control_pub.publish(control_msg);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //----------------------CONTROL MODE CHANGE HANDLING------------------------
    //switch control modes if controller button is pressed
    if (mode_change_requested)
    {

      //reset mode change requested to prevent mode from toggling twice on one button press
      mode_change_requested = false;

      //wait for verification from firing program that it's safe to change modes
      if (!disable_firing_clt.call(disable_firing_srv))
      {
        ROS_INFO("[control_node] failed to call disable firing service");
        ROS_BREAK();
      }

      //wait for verification from firing program that it's safe to change modes
      if (!disable_line_following_clt.call(disable_line_following_srv))
      {
        ROS_INFO("[control_node] failed to call disable line following service");
        ROS_BREAK();
      }

      //wait for verification from firing program that it's safe to change modes
      if (!disable_manual_control_clt.call(disable_manual_control_srv))
      {
        ROS_INFO("[control_node] failed to call disable manual control service");
        ROS_BREAK();
      }

      //notify of any busy nodes
      if (!disable_firing_srv.response.ready_to_change)
        ROS_INFO("[control_node] firing node not ready for mode change");
      if (!disable_line_following_srv.response.ready_to_change)
        ROS_INFO("[control_node] line following node not ready for mode change");
      if (!disable_manual_control_srv.response.ready_to_change)
        ROS_INFO("[control_node] manual control node not ready for mode change");


      if (disable_firing_srv.response.ready_to_change && disable_line_following_srv.response.ready_to_change && disable_manual_control_srv.response.ready_to_change)
      {

        //switch control modes
        autonomous_control = !autonomous_control;

        //set time and parameters of control message
        //when autonomous control is activated, robot is invariably returned to navigation mode
        //this is to ensure robot does not enter firing mode unless navigation has been complete
        control_msg.header.stamp = ros::Time::now();
        control_msg.autonomous_control = autonomous_control;
        control_msg.complete = false;
        control_msg.firing_stage = false;
        control_msg.navigation_stage = true;

        //publish control message
        control_pub.publish(control_msg);

        //disable component motor driver standby mode
        digitalWrite(component_motor_standby_pin, HIGH);

        //if autonomous control is enabled then ensure all hardware except firing motor is enabled
        if (autonomous_control)
        {

          //set time and parameters of conveyor motor message
          conveyor_msg.header.stamp = ros::Time::now();
          conveyor_msg.enable = true;

          //set time and parameters of firing motor message
          firing_motor_msg.header.stamp = ros::Time::now();
          firing_motor_msg.enable = false;

          //set time and parameters of roller motor message
          roller_msg.header.stamp = ros::Time::now();
          roller_msg.enable = true;

          //publish motor messages
          conveyor_pub.publish(conveyor_msg);
          firing_motor_pub.publish(firing_motor_msg);
          roller_pub.publish(roller_msg);

          //output ROS_INFO messages to inform of autonomous mode enabled
          ROS_DEBUG("[control_mode_node] control mode changed, autonomous control enabled");
          ROS_DEBUG("[control_mode_node] conveyor and roller motors enabled, firing motor disabled");

        }
        //if autonomous control is disabled leave hardware status unchanged
        else
        {

          //set time and parameters of conveyor motor message
          conveyor_msg.header.stamp = ros::Time::now();
          conveyor_msg.enable = false;

          //set time and parameters of firing motor message
          firing_motor_msg.header.stamp = ros::Time::now();
          firing_motor_msg.enable = false;

          //set time and parameters of roller motor message
          roller_msg.header.stamp = ros::Time::now();
          roller_msg.enable = false;

          //publish motor messages
          conveyor_pub.publish(conveyor_msg);
          firing_motor_pub.publish(firing_motor_msg);
          roller_pub.publish(roller_msg);

          //output ROS_INFO messages to inform of autonomous mode disabled
          ROS_INFO("[control_mode_node] control mode changed, autonomous control disabled");

        }

      }

    }

    //---------------[AUTONOMOUS MODE] END OF LINE REACHED HANDLING-------------
    //if autonomous mode is enabled, line following is complete, and robot is still in navigation stage:
    //line following has just been complete; disable navigation and switch to firing stage
    //conveyor and roller motors and their driver are disabled in firing stage to conserve power
    //firing motor must be enabled if it is not already
    if (autonomous_control && line_following_complete && control_msg.navigation_stage)
    {

      //set line following complete to false to ensure rechecking if mode changes
      line_following_complete = false;

      //set time and parameters of control message
      control_msg.header.stamp = ros::Time::now();
      control_msg.autonomous_control = autonomous_control;
      control_msg.complete = false;
      control_msg.firing_stage = true;
      control_msg.navigation_stage = false;

      //publish control message
      control_pub.publish(control_msg);

      //if conveyor is enabled then disable it
      if (conveyor_msg.enable)
      {

        //set time and parameters of conveyor message
        conveyor_msg.header.stamp = ros::Time::now();
        conveyor_msg.enable = false;

        //publish conveyor motor message
        conveyor_pub.publish(conveyor_msg);
      }

      //if roller is enabled then disable it
      if (roller_msg.enable)
      {

        //set time and parameters of roller message
        roller_msg.header.stamp = ros::Time::now();
        roller_msg.enable = false;

        //publish roller motor message
        roller_pub.publish(roller_msg);

      }

      //enable component motor driver standby mode
      digitalWrite(component_motor_standby_pin, LOW);

      //enable firing motor if not already enabled
      if (!firing_motor_msg.enable)
      {

        //set time and parameters of firing motor message
        firing_motor_msg.header.stamp = ros::Time::now();
        firing_motor_msg.enable = true;

        //publish firing motor message
        firing_motor_pub.publish(firing_motor_msg);

      }

      //output ROS_INFO message to inform end of line was reached and of hardware status
      ROS_INFO("[control_mode_node] end of line reached, disabling conveyor and roller motors if enabled");

    }

    //-------------[AUTONOMOUS MODE] END OF FIRING ROUTINE HANDLING-------------
    //if autonomous mode is enabled, firing stage is complete, and robot is still in firing stage:
    //firing has just been complete; disable firing and switch to complete status
    //firing motor is disabled to conserve power
    if (autonomous_control && firing_complete && control_msg.firing_stage)
    {

      //set time and parameters of control message
      control_msg.header.stamp = ros::Time::now();
      control_msg.autonomous_control = autonomous_control;
      control_msg.complete = true;
      control_msg.firing_stage = false;
      control_msg.navigation_stage = false;

      //set firing complete to false to ensure rechecking if mode changes
      firing_complete = false;

      //publish control message
      control_pub.publish(control_msg);

      //disable firing wheel motor if enabled
      if (firing_motor_msg.enable)
      {

        //set time and parameters of firing motor message
        firing_motor_msg.header.stamp = ros::Time::now();
        firing_motor_msg.enable = false;

        //publish message
        firing_motor_pub.publish(firing_motor_msg);

      }

      //output ROS_INFO message to inform end firing is complete and of hardware status
      ROS_INFO("[control_mode_node] firing routine complete, disabling firing wheel motor");

      //output ROS_INFO message to inform run routine is complete
      ROS_INFO("[control_mode_node] normal operation routine finished; standing by");

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
