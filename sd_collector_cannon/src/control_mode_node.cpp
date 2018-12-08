//control mode node
//handles all tasks involved in switching between manual and autonomous control modes
#include <ros/ros.h>
#include <sd_msgs/ComponentMotor.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/FiringStatus.h>
#include <sd_msgs/LineFollowing.h>
#include <sd_msgs/Mosfet.h>
#include <signal.h>
#include <wiringPi.h>

//global variables
bool firing_completed = false;
bool line_following_completed = false;

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

//callback function called to process messages on firing status topic
void firingStatusCallback(const sd_msgs::FiringStatus::ConstPtr& msg)
{

  //set local value to match message value
  if (firing_completed != msg->completed)
    firing_completed = msg->completed;

}

//callback function called to process messages on line following topic
void lineFollowingCallback(const sd_msgs::LineFollowing::ConstPtr& msg)
{

  //set local value to match message value
  if (line_following_completed != msg->completed)
    line_following_completed = msg->completed;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting control_mode_node");

  //initialize node and create node handler
  ros::init(argc, argv, "control_mode_node");
  ros::NodeHandle node_private("~");

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve component from parameter server
  if (!node_private.getParam("/component_motor_driver/standby_pin", component_motor_standby_pin))
  {
    ROS_ERROR("[control_mode_node] component motor driver standby pin not defined in config file: sd_bringup/config/global.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/collector_cannon/control_mode_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[control_mode_node] control mode node refresh rate not defined in config file: sd_collector_cannon/config/collector_cannon.yaml");
    ROS_BREAK();
  }

  //retrieve toggle button pin from parameter server
  int toggle_button_pin;
  if (!node_private.getParam("/collector_cannon/control_mode_node/toggle_button_pin", toggle_button_pin))
  {
    ROS_ERROR("[control_mode_node] toggle button pin not defined in config file: sd_collector_cannon/config/collector_cannon.yaml");
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
  ros::Publisher control_pub = node_private.advertise<sd_msgs::Control>("control", 10, true);

  //create publisher to publish conveyor motor message with buffer size 10, and latch set to true
  ros::Publisher conveyor_pub = node_private.advertise<sd_msgs::ComponentMotor>("conveyor_motor", 10, true);

  //create publisher to publish firing wheel motor message with buffer size 10, and latch set to true
  ros::Publisher firing_motor_pub = node_private.advertise<sd_msgs::Mosfet>("firing_motor", 10, true);

  //create publisher to publish roller motor message with buffer size 10, and latch set to true
  ros::Publisher roller_pub = node_private.advertise<sd_msgs::ComponentMotor>("roller_motor", 10, true);

  //create subscriber to subscribe to firing status messages message topic with queue size set to 1000
  ros::Subscriber firing_status_sub = node_private.subscribe("firing_status", 1000, firingStatusCallback);

  //create subscriber to subscribe to line following messages message topic with queue size set to 1000
  ros::Subscriber line_following_sub = node_private.subscribe("line_following", 1000, lineFollowingCallback);

  //run wiringPi GPIO setup function and set pin modes
  wiringPiSetup();
  pinMode(toggle_button_pin, INPUT);

  //create variable for status of autonomous control
  bool autonomous_control = false;

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set toggle_button_status to current status of toggle toggle_button_pin
    //toggle_button_pin HIGH indicates autonomous control is ON
    //toggle_button_pin LOW indicates autonomous control is OFF (enable manual control)
    bool toggle_button_status = digitalRead(toggle_button_pin);

    //CONTROL MODE CHANGE HANDLING
    //switch control modes if current mode differs from mode requested by toggle button
    if (autonomous_control != toggle_button_status)
    {

      //switch control modes
      autonomous_control = toggle_button_status;

      //disable component motor driver standby mode
      digitalWrite(component_motor_standby_pin, HIGH);

      //set time and parameters of control message
      control_msg.header.stamp = ros::Time::now();
      control_msg.autonomous_control = toggle_button_status;

      //set time and parameters of conveyor message
      conveyor_msg.header.stamp = ros::Time::now();
      conveyor_msg.enable = toggle_button_status;

      //set time and parameters of firing motor message
      firing_motor_msg.header.stamp = ros::Time::now();
      firing_motor_msg.enable = toggle_button_status;

      //set time and parameters of roller message
      roller_msg.header.stamp = ros::Time::now();
      roller_msg.enable = toggle_button_status;

      //publish messages
      control_pub.publish(control_msg);
      conveyor_pub.publish(conveyor_msg);
      firing_motor_pub.publish(firing_motor_msg);
      roller_pub.publish(roller_msg);

      //output ROS_INFO messages to inform of change in control mode and hardware status
      if (autonomous_control)
      {
        ROS_INFO("[control_mode_node] control mode changed, autonomous control enabled");
        ROS_INFO("[control_mode_node] conveyor, firing wheel, and roller motors enabled");
      }
      else
      {
        ROS_INFO("[control_mode_node] control mode changed, autonomous control disabled");
        ROS_INFO("[control_mode_node] conveyor, firing wheel, and roller motors disabled");
      }

    }

    //[AUTONOMOUS MODE] END OF LINE REACHED HANDLING
    //if autonomous mode is enabled, line following is complete, and either conveyor or roller are enabled:
    //robot is switching to firing mode; disable conveyor and roller motors
    if (autonomous_control && line_following_completed && (conveyor_msg.enable || roller_msg.enable))
    {

      //set time and parameters of conveyor message
      conveyor_msg.header.stamp = ros::Time::now();
      conveyor_msg.enable = false;

      //set time and parameters of roller message
      roller_msg.header.stamp = ros::Time::now();
      roller_msg.enable = false;

      //publish messages
      conveyor_pub.publish(conveyor_msg);
      roller_pub.publish(roller_msg);

      //enable component motor driver standby mode
      digitalWrite(component_motor_standby_pin, LOW);

      //output ROS_INFO message to inform end of line was reached and of hardware status
      ROS_INFO("[control_mode_node] end of line reached, disabling conveyor and roller motors");

    }

    //[AUTONOMOUS MODE] END OF FIRING ROUTINE HANDLING
    //if autonous mode is enabled, line following is complete, firing routine is complete, and
    //firing motor message enable status is still true then motor has not yet been turned off:
    //robot has fired all balls; disable shooting wheel if enabled
    if (autonomous_control && line_following_completed && firing_completed && firing_motor_msg.enable)
    {

      //set time and parameters of firing motor message
      firing_motor_msg.header.stamp = ros::Time::now();
      firing_motor_msg.enable = false;

      //publish message
      firing_motor_pub.publish(firing_motor_msg);

      //output ROS_INFO message to inform end firing is complete and of hardware status
      ROS_INFO("[control_mode_node] firing routine complete, disabling firing wheel motor");

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
