// find_wall.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_wall");

  Find_Wall* obj = new Find_Wall();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Find_Wall::Find_Wall()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>("find_wall", 1);
  //com_sub = n.subscribe("", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Find_Wall::command()
{
  boost::mutex::scoped_lock lock(com_mutex);

  // calculate the output
  com.message = "FIND A WALL";
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
  com.vel.linear.y = -VEL_FIND;

  com_pub.publish(com);
}

