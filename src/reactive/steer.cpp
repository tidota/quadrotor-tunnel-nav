// steer.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "steer");

  Steer* obj = new Steer();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Steer::Steer()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_STR, 1);
  //list_com_sub[TOPIC_MID] = n.subscribe(TOPIC_MID, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Steer::command()
{
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com;
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;

  double rate = rng_h[7].range / rng_h[5].range;

  // input check
  if(rate > DIST_RATE_STRR)
  {
    com.message = "STEER TO THE RIGHT";
    // calculate the output
    com.vel.angular.z = -MAX_VEL_STEER * (rate - 1.0);
  }
  else if(rate < DIST_RATE_STRL)
  {
    com.message = "STEER TO THE LEFT";
    // calculate the output
    com.vel.angular.z = MAX_VEL_STEER * (1.0 - rate);
  }

  com_pub.publish(com);
}


