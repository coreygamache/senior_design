//ball firing logic node
#include <ros/ros.h>
#include <sd_msgs/BallsCollected.h>
#include <sd_msgs/FiringStatus.h>
#include <sd_msgs/LineFollowing.h>
#include <sd_msgs/Mosfet.h>
#include <signal.h>

//global variables
bool firing_motor_on = false;
bool line_following_completed = false;
bool ready_to_fire = true;
int balls_collected = 0;


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

//callback function called to process messages on firing motor topic
void firingMotorCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //set local value to match message value
  firing_motor_on = msg->enable;

}

//callback function called to process messages on line following topic
void lineFollowingCallback(const sd_msgs::LineFollowing::ConstPtr& msg)
{

    //set local value to match message value
    line_following_completed = msg->completed;

}

void timerCallback(const ros::TimerEvent& event)
{

  //specified time since last shot fired has elapsed
  //reset ready to fire to true
  ready_to_fire = true;

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

  //retrieve fire delay time from parameter server [ms]
  float fire_delay_time;
  if (!node_private.getParam("/collector_cannon/firing_node/fire_delay_time", fire_delay_time))
  {
    ROS_ERROR("[firing_node] fire delay time not defined in config file: sd_collector_cannon/config/collector_cannon.yaml");
    ROS_BREAK();
  }

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/collector_cannon/firing_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[firing_node] firing node refresh rate not defined in config file: sd_collector_cannon/config/collector_cannon.yaml");
    ROS_BREAK();
  }

  //create gate solenoid message object and set default parameters
  sd_msgs::FiringStatus firing_status_msg;
  firing_status_msg.header.frame_id = "0";
  firing_status_msg.balls_fired = 0;
  firing_status_msg.balls_remaining = 0;
  firing_status_msg.completed = false;

  //create gate solenoid message object and set default parameters
  sd_msgs::Mosfet gate_solenoid_msg;
  gate_solenoid_msg.header.frame_id = "0";
  gate_solenoid_msg.enable = true;
  gate_solenoid_msg.pwm = 0;

  //create publisher to publish control message status with buffer size 10, and latch set to true
  ros::Publisher firing_status_pub = node_private.advertise<sd_msgs::FiringStatus>("firing_status", 10, true);

  //create publisher to publish control message status with buffer size 10, and latch set to true
  ros::Publisher gate_solenoid_pub = node_private.advertise<sd_msgs::Mosfet>("gate_solenoid", 10, false);

  //create subscriber to subscribe to balls collected messages message topic with queue size set to 1000
  ros::Subscriber balls_collected_sub = node_private.subscribe("balls_collected", 1000, ballsCollectedCallback);

  //create subscriber to subscribe to firing motor messages message topic with queue size set to 1000
  ros::Subscriber firing_motor_sub = node_private.subscribe("firing_motor", 1000, firingMotorCallback);

  //create variable for counting number of balls remaining
  int balls_fired = 0;

  //create timer to keep tracking of fire delay times
  ros::Timer timer;

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if line following is completed, fire delay time has elapsed (since last shot if there was one),
    //firing motor is on, at least one ball is remaining, request gate solenoid to open to fire a ball
    //NOTE: line_following_completed can only be true if control mode is set to autonomous
    //NOTE: (see line_follower_node for logic)
    if (line_following_completed && ready_to_fire && firing_motor_on && ((balls_collected - balls_fired) > 0))
    {

      //publish gate solenoid message to request gate to be opened
      gate_solenoid_pub.publish(gate_solenoid_msg);

      //increment number of balls fired
      balls_fired++;

      //set ready to fire to false until fire delay time elapses
      ready_to_fire = false;

      //set timer to keep track of fire delay time
      timer = node_private.createTimer(ros::Duration(fire_delay_time), timerCallback, true);

      //inform that a ball was fired
      ROS_INFO("[firing_node] ball fired, balls remaining: %d", balls_collected - balls_fired);

    }
    else if (!firing_motor_on)
    {

      //inform that firing wheel motor is not enabled; do not fire
      ROS_INFO("[firing_node] firing wheel motor not enabled; waiting to fire");

    }

    //if number of balls fired or remaining has changed then publish new message
    if ((balls_fired != firing_status_msg.balls_fired) || ((balls_collected - balls_fired) != firing_status_msg.balls_remaining))
    {

      //update message values
      firing_status_msg.balls_fired = balls_fired;
      firing_status_msg.balls_remaining = balls_collected - balls_fired;

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
