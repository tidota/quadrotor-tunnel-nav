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
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_OBS, 1);
  list_com_sub[TOPIC_ALT] = n.subscribe(TOPIC_ALT, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
  list_com_sub[TOPIC_TRN] = n.subscribe(TOPIC_TRN, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
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
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com1 = list_com[TOPIC_ALT];
  quadrotor_tunnel_nav::Com com2 = list_com[TOPIC_TRN];

  quadrotor_tunnel_nav::Com com;
  com = com1.message + " | " + com2.message;
  com.vel.linear = com1.vel.linear + com2.vel.linear;
  com.vel.angular = com1.vel.angular + com2.vel.angular;

  // input check
  if(
    rng_h[0].range < DIST_OBS || rng_h[1].range < DIST_OBS || rng_h[2].range < DIST_OBS || rng_h[3].range < DIST_OBS ||
    rng_h[4].range < DIST_OBS || rng_h[5].range < DIST_OBS || rng_h[6].range < DIST_OBS || rng_h[7].range < DIST_OBS ||
    rng_u[0].range < DIST_OBS || rng_u[1].range < DIST_OBS || rng_u[2].range < DIST_OBS || 
    rng_d[0].range < DIST_OBS || rng_d[1].range < DIST_OBS || rng_d[2].range < DIST_OBS)
  {
    com.message = "OBSTACLE AVOIDANCE";
    com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    if(rng_h[0].range < DIST_OBS)
    {
      com.vel.linear.x -= VEL_OBS;
    }
    if(rng_h[1].range < DIST_OBS)
    {
      com.vel.linear.x -= VEL_OBS/sqrt(2);
      com.vel.linear.y -= VEL_OBS/sqrt(2);
    }
    if(rng_h[2].range < DIST_OBS)
    {
      com.vel.linear.y -= VEL_OBS;
    }
    if(rng_h[3].range < DIST_OBS)
    {
      com.vel.linear.x += VEL_OBS/sqrt(2);
      com.vel.linear.y -= VEL_OBS/sqrt(2);
    }
    if(rng_h[4].range < DIST_OBS)
    {
      com.vel.linear.y += VEL_OBS;
    }
    if(rng_h[5].range < DIST_OBS)
    {
      com.vel.linear.x += VEL_OBS/sqrt(2);
      com.vel.linear.y += VEL_OBS/sqrt(2);
    }
    if(rng_h[6].range < DIST_OBS)
    {
      com.vel.linear.y += VEL_OBS;
    }
    if(rng_h[7].range < DIST_OBS)
    {
      com.vel.linear.x -= VEL_OBS/sqrt(2);
      com.vel.linear.y += VEL_OBS/sqrt(2);
    }
    if(rng_u[0].range < DIST_OBS)
    {
      com.vel.linear.x -= VEL_OBS/sqrt(2);
      com.vel.linear.z -= VEL_OBS/sqrt(2);
    }
    if(rng_u[1].range < DIST_OBS)
    {
      com.vel.linear.z -= VEL_OBS;
    }
    if(rng_u[2].range < DIST_OBS)
    {
      com.vel.linear.x += VEL_OBS/sqrt(2);
      com.vel.linear.z -= VEL_OBS/sqrt(2);
    }
    if(rng_d[0].range < DIST_OBS)
    {
      com.vel.linear.x -= VEL_OBS/sqrt(2);
      com.vel.linear.z += VEL_OBS/sqrt(2);
    }
    if(rng_d[1].range < DIST_OBS)
    {
      com.vel.linear.z += VEL_OBS;
    }
    if(rng_d[2].range < DIST_OBS)
    {
      com.vel.linear.x += VEL_OBS/sqrt(2);
      com.vel.linear.z += VEL_OBS/sqrt(2);
    }
  }

  com_pub.publish(com);
}

