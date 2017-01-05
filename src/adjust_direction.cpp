// adjust_direction.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adjust_direction");

  Adjust_Direction* obj = new Adjust_Direction();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Adjust_Direction::Adjust_Direction()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("adjust_direction", 1);
  vel_sub = n.subscribe("middle_line", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Adjust_Direction::~Adjust_Direction()
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
void Adjust_Direction::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);

  // input check
  if(rng_h[7].range > rng_h[6].range * sqrt(2) * DIST_RATE_ADJR)
  {
    ROS_INFO("ADJUST THE DIRECTION TO THE RIGHT");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.angular.z = -VEL_TURN;
    vel.linear.x = VEL_STRAIGHT;
  }
  else if(rng_h[7].range < rng_h[6].range * sqrt(2) * DIST_RATE_ADJL)
  {
    ROS_INFO("ADJUST THE DIRECTION TO THE LEFT");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.angular.z = VEL_TURN;
	vel.linear.x = VEL_STRAIGHT;
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Adjust_Direction::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

