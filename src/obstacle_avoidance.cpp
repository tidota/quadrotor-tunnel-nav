// obstacle_avoidance.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance");

  Obs_Avoid* obj = new Obs_Avoid();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Obs_Avoid::Obs_Avoid()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("obstacle_avoidance", 1);
  vel_sub = n.subscribe("keep_altitude", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Obs_Avoid::~Obs_Avoid()
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
void Obs_Avoid::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;

  // input check
  if(
    rng_h[0].range < DIST_OBS || rng_h[1].range < DIST_OBS || rng_h[2].range < DIST_OBS || rng_h[3].range < DIST_OBS ||
    rng_h[4].range < DIST_OBS || rng_h[5].range < DIST_OBS || rng_h[6].range < DIST_OBS || rng_h[7].range < DIST_OBS ||
    rng_u[0].range < DIST_OBS || rng_u[1].range < DIST_OBS || rng_u[2].range < DIST_OBS || 
    rng_d[0].range < DIST_OBS || rng_d[1].range < DIST_OBS || rng_d[2].range < DIST_OBS)
  {
    ROS_INFO("OBSTACLE AVOIDANCE");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    if(rng_h[0].range < DIST_OBS)
    {
      vel.linear.x -= VEL_OBS;
    }
    if(rng_h[1].range < DIST_OBS)
    {
      vel.linear.x -= VEL_OBS/sqrt(2);
      vel.linear.y -= VEL_OBS/sqrt(2);
    }
    if(rng_h[2].range < DIST_OBS)
    {
      vel.linear.y -= VEL_OBS;
    }
    if(rng_h[3].range < DIST_OBS)
    {
      vel.linear.x += VEL_OBS/sqrt(2);
      vel.linear.y -= VEL_OBS/sqrt(2);
    }
    if(rng_h[4].range < DIST_OBS)
    {
      vel.linear.y += VEL_OBS;
    }
    if(rng_h[5].range < DIST_OBS)
    {
      vel.linear.x += VEL_OBS/sqrt(2);
      vel.linear.y += VEL_OBS/sqrt(2);
    }
    if(rng_h[6].range < DIST_OBS)
    {
      vel.linear.y += VEL_OBS;
    }
    if(rng_h[7].range < DIST_OBS)
    {
      vel.linear.x -= VEL_OBS/sqrt(2);
      vel.linear.y += VEL_OBS/sqrt(2);
    }
    if(rng_u[0].range < DIST_OBS)
    {
      vel.linear.x -= VEL_OBS/sqrt(2);
      vel.linear.z -= VEL_OBS/sqrt(2);
    }
    if(rng_u[1].range < DIST_OBS)
    {
      vel.linear.z -= VEL_OBS;
    }
    if(rng_u[2].range < DIST_OBS)
    {
      vel.linear.x += VEL_OBS/sqrt(2);
      vel.linear.z -= VEL_OBS/sqrt(2);
    }
    if(rng_d[0].range < DIST_OBS)
    {
      vel.linear.x -= VEL_OBS/sqrt(2);
      vel.linear.z += VEL_OBS/sqrt(2);
    }
    if(rng_d[1].range < DIST_OBS)
    {
      vel.linear.z += VEL_OBS;
    }
    if(rng_d[2].range < DIST_OBS)
    {
      vel.linear.x += VEL_OBS/sqrt(2);
      vel.linear.z += VEL_OBS/sqrt(2);
    }
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Obs_Avoid::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

