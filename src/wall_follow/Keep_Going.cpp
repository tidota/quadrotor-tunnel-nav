#include "wall_follow/Keep_Going.hpp"

// ============================================================================================
// Constructor
// ============================================================================================
Keep_Going::Keep_Going(const double _vel): Go_Straight(), vel(_vel)
{
  ros::NodeHandle n;
  this->vel_change_sub
    = n.subscribe("/nav_vel_update", 1, &Keep_Going::update_velocity, this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Keep_Going::command()
{
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com;
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;

  double rate = rng_h[7].range / sqrt(2.0) / rng_h[6].range;
  if (rate > 1.0)
    rate = 1.0;

  if (rng_h[0].range < 7.9)
    rate = rng_h[0].range / 8.0;
  //if (rng_h[0].range < DIST_WALL * 0.3)
  //  rate = 0.3;
  //if (rng_h[7].range < DIST_WALL * sqrt(2.0) * 0.9)
  //  rate = 0.1;

  // input check
  com.message = "KEEP GOING";
  // calculate the output
  com.vel.linear.x = this->vel * rate;

  com_pub.publish(com);
}

// ============================================================================================
// update_velocity
//
// callback for update of the velocity
// ============================================================================================
void Keep_Going::update_velocity(const std_msgs::Float32::ConstPtr& new_vel)
{
  this->vel = new_vel->data;
}
