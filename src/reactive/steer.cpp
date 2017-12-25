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
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>("steer", 1);
  com_sub = n.subscribe("middle_line", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
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

  // input check
  if(rng_h[7].range > rng_h[6].range * sqrt(2) * DIST_RATE_STRR)
  {
    com.message = com.message + " + STEER TO THE RIGHT";
    //com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    //com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    com.vel.angular.z += -VEL_TURN;
    //com.vel.linear.x += VEL_STRAIGHT;
  }
  else if(rng_h[7].range < rng_h[6].range * sqrt(2) * DIST_RATE_STRL)
  {
    com.message = com.message + " + STEER TO THE LEFT";
    //com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    //com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    com.vel.angular.z += VEL_TURN;
    //com.vel.linear.x += VEL_STRAIGHT;
  }

  com_pub.publish(com);
}


