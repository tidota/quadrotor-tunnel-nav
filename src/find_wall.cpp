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
  vel_pub = n.advertise<geometry_msgs::Twist>("find_wall", 1);
  //vel_sub = n.subscribe("", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Find_Wall::~Find_Wall()
{
  this->stop();
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
  boost::mutex::scoped_lock lock(vel_mutex);

  // calculate the output
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel.linear.y = -VEL_FIND;

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Find_Wall::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

