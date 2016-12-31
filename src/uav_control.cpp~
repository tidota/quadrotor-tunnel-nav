// uav_control.cpp
// 161130
// this is the control code for UAV in the simulated enviornment by gazebo.
// 

#include "uav_control.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_control");

  UAV_Control::create_control();

  ros::spin();

  UAV_Control::kill_control();

  return(0);
}

// ============================================================================================
// definitions of static members in the class
// ============================================================================================
UAV_Control* UAV_Control::p_control = NULL;

// ============================================================================================
// create_control
//
// To make the class singleton, this function creates a new object if there is no existing one.
// otherwise, it returns the existing object.
//
// note: signal funciton must be called
//       after the constructor is called where the node hander is created.
// ============================================================================================
UAV_Control *UAV_Control::create_control()
{
  if(UAV_Control::p_control == NULL)
  {
    // create a new object
    UAV_Control::p_control = new UAV_Control();

    // set up for signal handler
    signal(SIGINT,UAV_Control::quit);
  }
  return UAV_Control::p_control;
}

// ============================================================================================
// kill_control
//
// It releases the memotry after sending a command to stop the UAV
// ============================================================================================
void UAV_Control::kill_control()
{
  if(p_control != NULL)
  {
    p_control->stop(); // stops the UAV and it will stay on the current location.
    delete p_control;
    p_control = NULL;
  }
}

// ============================================================================================
// Constructor
// ============================================================================================
UAV_Control::UAV_Control()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  vel_sub = n.subscribe("obstacle_avoidance", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void UAV_Control::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void UAV_Control::stop()
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = -1.0;//0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  UAV_Control::vel_pub.publish(vel);
  timer.stop();
}

// ============================================================================================
// quit
//
// it is to be called when Ctrl-C is hit on the terminal.
// it kills the running control and releases the memory.
// ============================================================================================
void UAV_Control::quit(int sig)
{
  ROS_INFO("UAV Control: signal received, shutting down");
  UAV_Control::kill_control();
  ros::shutdown();
}



