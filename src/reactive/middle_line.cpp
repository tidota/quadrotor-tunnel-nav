// middle_line.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "middle_line");

  Middle_Line* obj = new Middle_Line();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Middle_Line::Middle_Line()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>("middle_line", 1);
  com_sub = n.subscribe("go_straight", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Middle_Line::command()
{
  boost::mutex::scoped_lock lock(com_mutex);

  // input check
  if(rng_h[2].range <= rng_h[0].range &&
  (rng_h[6].range < rng_h[2].range - DIST_OFF || rng_h[6].range > rng_h[2].range + DIST_OFF))
  {
    com.message = "STAY ON THE MIDDLE LINE";
    com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    com.vel.linear.y = (rng_h[6].range < rng_h[2].range)? VEL_MIDDLE: -VEL_MIDDLE;
  }

  com_pub.publish(com);
}

