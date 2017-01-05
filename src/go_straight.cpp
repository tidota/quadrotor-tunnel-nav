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
  vel_pub = n.advertise<geometry_msgs::Twist>("go_straight", 1);
  vel_sub = n.subscribe("find_wall", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Go_Straight::~Go_Straight()
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
void Go_Straight::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);

  // input check
  if(rng_h[6].range < DIST_MAX)
  {
    ROS_INFO("GO STRAIGHT");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.linear.x = VEL_STRAIGHT;
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Go_Straight::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

