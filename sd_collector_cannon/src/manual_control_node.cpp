//manual control node
//handles Dual Shock 4 controller input to manually control robot
#include <math.h>
#include <ros/ros.h>
#include <sd_msgs/ComponentMotor.h>
#include <sd_msgs/Control.h>
#include <sd_msgs/DriveMotors.h>
#include <sd_msgs/Mosfet.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>

//global variables
bool autonomous_control = false;
bool conveyor_enable = false;
bool firing_motor_enable = false;
bool roller_enable = false;

//global controller variables
std::vector<float> controller_axes(8, 0);
std::vector<int> controller_buttons(13, 0);


//callback function called to process SIGINT command
void sigintHandler(int sig)
{

  //call the default shutdown function
  ros::shutdown();

}

//callback function called to process messages on control topic
void controlCallback(const sd_msgs::Control::ConstPtr& msg)
{

  //set local value to match message value
  autonomous_control = msg->autonomous_control;

  if (!autonomous_control)
  {
    ROS_INFO("[manual_control_node] entering manual control mode");
  }

}

//callback function called to process messages on conveyor motor topic
void conveyorCallback(const sd_msgs::ComponentMotor::ConstPtr& msg)
{

  //set local value to match message value
  conveyor_enable = msg->enable;

}

//callback function called to process messages on joy topic
void controllerCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

  //set local values to match message values
  controller_axes = msg->axes;
  controller_buttons = msg->buttons;

}

//callback function called to process messages on firing wheel motor topic
void firingMotorCallback(const sd_msgs::Mosfet::ConstPtr& msg)
{

  //set local value to match message value
  firing_motor_enable = msg->enable;

}

//callback function called to process messages on roller motor topic
void rollerCallback(const sd_msgs::ComponentMotor::ConstPtr& msg)
{

  //set local value to match message value
  roller_enable = msg->enable;

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting manual_control_node");

  //initialize node and create node handler
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle node_private("~");
  ros::NodeHandle node_public;

  //override the default SIGINT handler
  signal(SIGINT, sigintHandler);

  //retrieve refresh rate of node in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("/control/manual_control_node/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[manual_control_node] manual control node refresh rate not defined in config file: sd_collector_cannon/config/control.yaml");
    ROS_BREAK();
  }

  //create conveyor message object and set default parameters
  sd_msgs::ComponentMotor conveyor_msg;
  conveyor_msg.header.frame_id = "0";
  conveyor_msg.enable = conveyor_enable;
  conveyor_msg.direction = 0;
  conveyor_msg.pwm = 0;

  //create drive motors message and set default parameters
  sd_msgs::DriveMotors drive_motors_msg;
  drive_motors_msg.header.frame_id = "0";
  drive_motors_msg.left_motor_dir = 0;
  drive_motors_msg.right_motor_dir = 0;
  drive_motors_msg.left_motor_pwm = 0;
  drive_motors_msg.right_motor_pwm = 0;

  //create firing motor message object and set default parameters
  sd_msgs::Mosfet firing_motor_msg;
  firing_motor_msg.header.frame_id = "0";
  firing_motor_msg.enable = firing_motor_enable;
  firing_motor_msg.pwm = 0;

  //create gate solenoid message object and set default parameters
  //the firing node automatically disables the solenoid after a set amount of time,
  //therefore this node only needs to send a message with enable = true
  //when a ball is to be fired
  sd_msgs::Mosfet gate_solenoid_msg;
  gate_solenoid_msg.header.frame_id = "0";
  gate_solenoid_msg.enable = true;
  gate_solenoid_msg.pwm = 0;

  //create roller motor message object and set default parameters
  sd_msgs::ComponentMotor roller_msg;
  roller_msg.header.frame_id = "0";
  roller_msg.enable = firing_motor_enable;
  roller_msg.direction = 0;
  roller_msg.pwm = 0;

  //create publisher to publish conveyor motor message with buffer size 10, and latch set to true
  ros::Publisher conveyor_pub = node_public.advertise<sd_msgs::ComponentMotor>("conveyor_motor", 10, true);

  //create publisher to publish drive motors message with buffer size 10, and latch set to true
  ros::Publisher drive_motors_pub = node_public.advertise<sd_msgs::DriveMotors>("drive_motors", 10, true);

  //create publisher to publish firing wheel motor message with buffer size 10, and latch set to true
  ros::Publisher firing_motor_pub = node_public.advertise<sd_msgs::Mosfet>("firing_motor", 10, true);

  //create publisher to publish control message status with buffer size 10, and latch set to true
  ros::Publisher gate_solenoid_pub = node_public.advertise<sd_msgs::Mosfet>("gate_solenoid", 10, false);

  //create publisher to publish roller motor message with buffer size 10, and latch set to true
  ros::Publisher roller_pub = node_public.advertise<sd_msgs::ComponentMotor>("roller_motor", 10, true);

  //create subscriber to subscribe to control messages topic with queue size set to 1000
  ros::Subscriber control_sub = node_public.subscribe("control", 1000, controlCallback);

  //create subscriber to subscribe to conveyor motor messages topic with queue size set to 1000
  ros::Subscriber conveyor_sub = node_public.subscribe("conveyor_motor", 1000, conveyorCallback);

  //create subscriber to subscribe to joy messages topic with queue size set to 1000
  ros::Subscriber controller_sub = node_public.subscribe("joy", 1000, controllerCallback);

  //create subscriber to subscribe to firing wheel motor messages topic with queue size set to 1000
  ros::Subscriber firing_motor_sub = node_public.subscribe("firing_motor", 1000, firingMotorCallback);

  //create subscriber to subscribe to roller motor messages topic with queue size set to 1000
  ros::Subscriber roller_sub = node_public.subscribe("roller_motor", 1000, rollerCallback);

  //set loop rate in Hz
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //if autonomous control is disabled then allow manual control
    if (!autonomous_control)
    {

      //set drive motor message values as requested by controller input and publish message

      //set time of drive motors message
      drive_motors_msg.header.stamp = ros::Time::now();

      //set motor directions (0 = forward, 1 = reverse)
      if (controller_axes[1] >= 0)
        drive_motors_msg.left_motor_dir = 0;
      else
        drive_motors_msg.left_motor_dir = 1;

      if (controller_axes[4] >= 0)
        drive_motors_msg.right_motor_dir = 0;
      else
        drive_motors_msg.right_motor_dir = 1;

      //set motor PWM values
      drive_motors_msg.left_motor_pwm = 255 * fabs(controller_axes[1]);
      drive_motors_msg.right_motor_pwm = 255 * fabs(controller_axes[4]);

      //publish drive motors message
      drive_motors_pub.publish(drive_motors_msg);

      //enable/disable other motors as requested by controller input

      //if conveyor button on controller is pressed then change enable status and publish message
      if (controller_buttons[3] == 1)
      {

        //set button status to 0 to prevent consecutive toggles for one button press
        controller_buttons[3] = 0;

        //change conveyor motor enable status
        conveyor_enable = !conveyor_enable;

        //set time and parameters of conveyor motor message
        conveyor_msg.header.stamp = ros::Time::now();
        conveyor_msg.enable = conveyor_enable;

        //publish conveyor motor message
        conveyor_pub.publish(conveyor_msg);

        //output ROS_INFO messages to inform of conveyor enable status change
        ROS_INFO("[manual_control_node] conveyor motor enable status changed: %d", conveyor_enable);

      }

      //if fire button on controller is pressed then publish fire request message
      if (controller_buttons[7] == 1)
      {

        //set button status to 0 to prevent consecutive toggles for one button press
        controller_buttons[7] = 0;

        //set time and parameters of firing motor message
        gate_solenoid_msg.header.stamp = ros::Time::now();

        //publish firing motor message
        gate_solenoid_pub.publish(gate_solenoid_msg);

        //output ROS_INFO messages to inform of fire ball request
        ROS_INFO("[manual_control_node] fire ball request issued");

      }

      //if firing wheel button on controller is pressed then change enable status and publish message
      if (controller_buttons[0] == 1)
      {

        //set button status to 0 to prevent consecutive toggles for one button press
        controller_buttons[0] = 0;

        //change firing wheel motor enable status
        firing_motor_enable = !firing_motor_enable;

        //set time and parameters of firing motor message
        firing_motor_msg.header.stamp = ros::Time::now();
        firing_motor_msg.enable = firing_motor_enable;

        //publish firing motor message
        firing_motor_pub.publish(firing_motor_msg);

        //output ROS_INFO messages to inform of firing motor enable status change
        ROS_INFO("[manual_control_node] firing wheel motor enable status changed: %d", firing_motor_enable);

      }

      //if roller button on controller is pressed then change enable status and publish message
      if (controller_buttons[1] == 1)
      {

        //set button status to 0 to prevent consecutive toggles for one button press
        controller_buttons[1] = 0;

        //change firing wheel motor enable status
        roller_enable = !roller_enable;

        //set time and parameters of roller motor message
        roller_msg.header.stamp = ros::Time::now();
        roller_msg.enable = roller_enable;

        //publish roller motor message
        roller_pub.publish(roller_msg);

        //output ROS_INFO messages to inform of roller enable status change
        ROS_INFO("[manual_control_node] roller motor enable status changed: %d", roller_enable);

      }

    }

    //process callback function calls
    ros::spinOnce();

    //sleep until next cycle
    loop_rate.sleep();
  }

  return 0;
}
