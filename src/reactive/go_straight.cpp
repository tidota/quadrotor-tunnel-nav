// go_straight.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_straight");

  Go_Straight* obj = new Go_Straight();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Go_Straight::Go_Straight()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>("go_straight", 1);
  com_sub = n.subscribe("steer", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Go_Straight::command()
{
  boost::mutex::scoped_lock lock(com_mutex);

  // input check
  com.message = "GO STRAIGHT";
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
  // calculate the output
  com.vel.linear.x = VEL_STRAIGHT;

  com_pub.publish(com);
}

