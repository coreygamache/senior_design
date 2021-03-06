//gate servo control node (servoblaster version)
#include <iostream> //dependency for fstream (must be included first)
#include <fstream>
#include <errno.h>
#include <ros/ros.h>
#include <sd_msgs/GateServo.h>
#include <signal.h>

//global variables
bool gate_open = false; //servo angle open status
bool open_requested = false;
int closed_angle;
int open_angle;
int sb_servo_number;
std::string sb_driver_path;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on gate_servo topic
void gateServoCallback(const sd_msgs::GateServo::ConstPtr& msg)
{

  //set local value to match message value
  if (msg->open)
    open_requested = true;

}

//callback function to process timer firing event
//specified time since gate was opened has elapsed; close gate
void openTimerCallback(const ros::TimerEvent& event)
{

  //inform of gate closure
  ROS_INFO("[gate_servo_sb_node] gate open time elapsed; returning servo to home position");

  //open servo driver
  std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

  //output angle change command to servo driver
  sb_driver << sb_servo_number << "=" << closed_angle << "\n";

  //close file
  sb_driver.close();

  //change gate open status
  gate_open = false;

}

//callback function to process timer firing event
//specified time since gate was opened has elapsed; close gate
void periodicTimerCallback(const ros::TimerEvent& event)
{

  //inform of gate closure
  ROS_DEBUG("[gate_servo_sb_node] outputting periodic signal to servo");

  //open servo driver
  std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

  //re-output current open/closed angle command to servo driver
  if (gate_open)
    sb_driver << sb_servo_number << "=" << open_angle << "\n";
  else
    sb_driver << sb_servo_number << "=" << closed_angle << "\n";

  //close file
  sb_driver.close();

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting gate_servo_sb_node");

  //initialize node and create node handler
  ros::init(argc, argv, "gate_servo_sb_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve gate closed angle value from parameter server [us]
  if (!node_private.getParam("/hardware/gate_servo_sb_node/closed_angle", closed_angle))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo closed angle not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //translate value in microseconds to units of 10 microseconds (required by servoblaster)
  closed_angle = closed_angle / 10;

  //retrieve gate open angle value from parameter server [us]
  if (!node_private.getParam("/hardware/gate_servo_sb_node/open_angle", open_angle))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo open angle not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //translate value in microseconds to units of 10 microseconds (required by servoblaster)
  open_angle = open_angle / 10;

  //retrieve gate open time value from parameter server [s]
  float open_time;
  if (!node_private.getParam("/hardware/gate_servo_sb_node/open_time", open_time))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo open time not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve gate periodic output time value from parameter server [s]
  float periodic_time;
  if (!node_private.getParam("/hardware/gate_servo_sb_node/periodic_time", periodic_time))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo periodic time not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of servo gate node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/hardware/gate_servo_sb_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo sb node refresh rate not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster driver path from parameter server
  if (!node_private.getParam("/hardware/gate_servo_sb_node/sb_driver_path", sb_driver_path))
  {
    ROS_ERROR("[gate_servo_sb_node] servoblaster driver path not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //retrieve servoblaster servo number from parameter server
  if (!node_private.getParam("/hardware/gate_servo_sb_node/sb_servo_number", sb_servo_number))
  {
    ROS_ERROR("[gate_servo_sb_node] gate servo number not defined in config file: sd_hardware_interface/config/hardware_interface.yaml");
    ROS_BREAK();
  }

  //create subscriber to subscribe to gate servo messages message topic with queue size set to 1000
  ros::Subscriber gate_servo_sub = node_public.subscribe("gate_servo", 1000, gateServoCallback);

  //create timer to keep track of gate open time
  ros::Timer open_timer;

  //create timer to periodically re-output signal to servo to prevent timeout
  ros::Timer periodic_timer;

  //initialize periodic timer
  periodic_timer = node_private.createTimer(ros::Duration(periodic_time), periodicTimerCallback, false);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if servo gate has been requested to open since last iteration then handle request
    if (open_requested && !gate_open)
    {

      //set open to false until next gate open request is received
      open_requested = false;

      //set open status of gate to true
      gate_open = true;

      //inform of gate open request
      ROS_INFO("[gate_servo_sb_node] gate open request received; opening gate servo");

      //open servo driver
      std::fstream sb_driver(sb_driver_path.c_str(), std::fstream::out | std::fstream::trunc);

      //output angle change command to servo driver
      sb_driver << sb_servo_number << "=" << open_angle << "\n";

      //close file
      sb_driver.close();

      //set timer to keep track of gate open time
      open_timer = node_private.createTimer(ros::Duration(open_time), openTimerCallback, true);

    }
    //if gate open was requested but gate is already open inform user
    else if (open_requested && gate_open)
    {

      //inform of problem
      ROS_INFO("[gate_servo_sb_node] gate open request received but gate already open; ignoring request");

      //reset open request to false
      open_requested = false;

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
