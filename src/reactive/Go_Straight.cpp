// go_straight.cpp

#include "reactive/Go_Straight.hpp"

// ============================================================================================
// Constructor
// ============================================================================================
Go_Straight::Go_Straight()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_GO, 1);
  //list_com_sub[TOPIC_STR] = n.subscribe(TOPIC_STR, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
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
  quadrotor_tunnel_nav::Com com;
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;

  double range = rng_h[0].range;
  if(range > DIST_MAX)
    range = DIST_MAX;
  double rate = range/DIST_MAX;

  // input check
  com.message = "GO STRAIGHT";
  // calculate the output
  com.vel.linear.x = (MAX_VEL_STRAIGHT - MIN_VEL_STRAIGHT) * rate + MIN_VEL_STRAIGHT;

  com_pub.publish(com);
}
