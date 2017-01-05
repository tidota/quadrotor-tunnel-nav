// obstacle_avoidance.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "keep_altitude");

  Keep_Alt* obj = new Keep_Alt();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Keep_Alt::Keep_Alt()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<uav_practice161129::Com>("keep_altitude", 1);
  com_sub = n.subscribe("turn", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Keep_Alt::command()
{
  boost::mutex::scoped_lock lock(com_mutex);

  // input check
  if(rng_u[1].range - rng_d[1].range > DIST_OFF_ALT || rng_d[1].range - rng_u[1].range > DIST_OFF_ALT)
  {
    com.message = "KEEP THE ALTITUDE";
    com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    com.vel.linear.z = VEL_ALT * (rng_d[1].range < rng_u[1].range)? 1: -1;
    //if(rng_h[2] > rng_h[0])
      //com.vel.linear.x = VEL_STRAIGHT;
  }

  com_pub.publish(com);
}

