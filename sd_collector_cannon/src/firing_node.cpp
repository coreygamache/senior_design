//ball firing logic node
#include <ros/ros.h>
#include <sd_msgs/BallsCollected.h>
#include <sd_msgs/ChangeControlMode.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/FiringStatus.h>
#include <sd_msgs/GateServo.h>
#include <sd_msgs/Mosfet.h>
#include <signal.h>

//global variables
bool autonomous_control = false;
bool firing_complete = false;
bool firing_motor_on = false;
bool firing_stage = false;
bool ready_to_fire = false;
float fire_delay_time;
int balls_collected = 0;
int balls_fired = 0;

//create timer to keep tracking of fire delay times
ros::Timer timer;


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on balls collected topic
void ballsCollectedCallback(const sd_msgs::BallsCollected::ConstPtr& msg)
{

  //if number of balls collected in message is valid then set local value to match
  if (msg->balls_collected >= 0)
    balls_collected = msg->balls_collected;

}

//callback function called to process messages on control topic
void controlCallback(const sd_msgs::Control::ConstPtr& msg)
{

  //change local control mode to match message
  autonomous_control = msg->autonomous_control;

  //if firing stage changes from false to true reset all firing values
  if (!firing_stage && msg->firing_stage)
  {

    //reset variables
    firing_complete = false;
    balls_collected = 3;
    balls_fired = 0;

  }

  //set local value to match message value
  firing_stage = msg->firing_stage;

}

//callback function called to process service requests on the disable firing topic
bool DisableFiringCallback(sd_msgs::ChangeControlMode::Request& req, sd_msgs::ChangeControlMode::Response& res)
{

  //if node isn't currently busy then ready to change modes, otherwise not ready to change
  res.ready_to_change = true;

  //output ROS INFO message to inform of mode change request and reply status
  if (req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[firing_node] mode change requested; changing control modes");
  else if (!req.mode_change_requested && res.ready_to_change)
    ROS_INFO("[firing_node] ready to change modes status requested; indicating ready to change");
  else if (req.mode_change_requested && !res.ready_to_change)
    ROS_INFO("[firing_node] mode change requested; indicating node is busy");
  else
    ROS_INFO("[firing_node] ready to change modes status requested; indicating node is busy");

  //return true to indicate service processing is complete
  return true;

}

//callback function called to process messages on firing motor topic
void firingMotorCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //set local value to match message value
  firing_motor_on = msg->enable;

  if (firing_motor_on)
    timer = node_private.createTimer(ros::Duration(fire_delay_time), timerCallback, true);
  else
    ready_to_fire = false;

}

//callback function to process timer firing event
void timerCallback(const ros::TimerEvent& event)
{

  //specified time since last shot fired has elapsed
  //reset ready to fire to true
  ready_to_fire = true;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting firing_node");

  //initialize node and create node handler
  ros::init(argc, argv, "firing_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve fire delay time from parameter server [ms]
  if (!node_private.getParam("/control/firing_node/fire_delay_time", fire_delay_time))
  {
    ROS_ERROR("[firing_node] fire delay time not defined in config file: sd_collector_cannon/config/control.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/firing_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[firing_node] firing node refresh rate not defined in config file: sd_collector_cannon/config/control.yaml");
    ROS_BREAK();
  }

  //create firing status message object and set default parameters
  sd_msgs::FiringStatus firing_status_msg;
  firing_status_msg.header.frame_id = "0";
  firing_status_msg.balls_fired = 0;
  firing_status_msg.balls_remaining = balls_collected;
  firing_status_msg.complete = false;

  //create gate servo message object and set default parameters
  sd_msgs::GateServo gate_servo_msg;
  gate_servo_msg.header.frame_id = "0";
  gate_servo_msg.open = true;

  //create publisher to publish firing status message with buffer size 10, and latch set to true
  ros::Publisher firing_status_pub = node_public.advertise<sd_msgs::FiringStatus>("firing_status", 10, true);

  //create publisher to publish gate servo message with buffer size 10, and latch set to true
  ros::Publisher gate_servo_pub = node_public.advertise<sd_msgs::GateServo>("gate_servo", 10, false);

  //create service to process service requests on the disable firing topic
  ros::ServiceServer disable_firing_srv = node_public.advertiseService("disable_firing", DisableFiringCallback);

  //create subscriber to subscribe to balls collected messages topic with queue size set to 1000
  ros::Subscriber balls_collected_sub = node_public.subscribe("/sensor/balls_collected", 1000, ballsCollectedCallback);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("control", 1000, controlCallback);

  //create subscriber to subscribe to firing motor messages topic with queue size set to 1000
  ros::Subscriber firing_motor_sub = node_public.subscribe("/hardware/firing_motor", 1000, firingMotorCallback);

  //publish initial firing status message
  firing_status_pub.publish(firing_status_msg);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if line following is complete, fire delay time has elapsed (since last shot if there was one),
    //firing motor is on, at least one ball is remaining, request gate servo to open to fire a ball
    //NOTE: line_following_complete can only be true if control mode is set to autonomous
    if (autonomous_control && firing_stage && ready_to_fire && firing_motor_on) // && ((balls_collected - balls_fired) > 0))
    {

      //publish gate servo message to request gate to be opened
      gate_servo_pub.publish(gate_servo_msg);

      //increment number of balls fired if there was a ball to fire
      if ((balls_collected - balls_fired) > 0)
        balls_fired += 3;

      //set ready to fire to false until fire delay time elapses
      ready_to_fire = false;

      //set timer to keep track of fire delay time
      timer = node_private.createTimer(ros::Duration(fire_delay_time), timerCallback, true);

      //inform that a ball was fired
      ROS_INFO("[firing_node] ball fired, balls remaining: %d", balls_collected - balls_fired);

    }
    //if all of above conditions are met except the firing motor isn't on then inform of status without firing
    else if (autonomous_control && firing_stage && ready_to_fire && !firing_motor_on && ((balls_collected - balls_fired) > 0))
    {

      //inform that firing wheel motor is not enabled; do not fire
      ROS_INFO("[firing_node] firing wheel motor not enabled; waiting to fire");

    }

    //if currently in firing stage and no balls remain then firing is complete; set message value
    if (firing_stage && ((balls_collected - balls_fired) == 0))
      firing_complete = true;
    else
      firing_complete = false;

    //if number of balls fired, balls remaining, or complete status has changed then publish new message
    if ((balls_fired != firing_status_msg.balls_fired) || ((balls_collected - balls_fired) != firing_status_msg.balls_remaining) || (firing_complete != firing_status_msg.complete))
    {

      //update message values
      firing_status_msg.balls_fired = balls_fired;
      firing_status_msg.balls_remaining = balls_collected - balls_fired;
      firing_status_msg.complete = firing_complete;

      //publish new message
      firing_status_pub.publish(firing_status_msg);

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
