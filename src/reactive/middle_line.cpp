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
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_MID, 1);
  //com_sub = n.subscribe("", 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
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
  quadrotor_tunnel_nav::Com com;
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;

  double lengL = (rng_h[2].range < rng_h[1].range/sqrt(2))? rng_h[2].range: rng_h[1].range/sqrt(2);
  double lengR = rng_h[6].range;
  double lengF = rng_h[0].range;

  // diff_rate is the gap from the mid with respect to y axis (left is positive)
  double mid_leng = (lengL + lengR)/2;
  double diff_leng = (lengR - lengL)/2;
  double diff_rate = (mid_leng != 0)? diff_leng/mid_leng: 0;

  // input check
  // if the front side is clear and it is out of range from the middle line
  if(diff_rate < -DIST_OFF_RATE_MID || DIST_OFF_RATE_MID < diff_rate)
  {
    com.message = "STAY ON THE MIDDLE LINE";
    com.vel.linear.y += (lengR < lengL)? MAX_VEL_MID: -MAX_VEL_MID;
  }
  // if it is slightly off the mid line, apply a proportional value
  else if(diff_rate < -DIST_OFF_RATE_MID*0.5 || DIST_OFF_RATE_MID*0.5 < diff_rate)
  {
    com.message = "STAY ON THE MIDDLE LINE";
    com.vel.linear.y -= MAX_VEL_MID * diff_rate / DIST_OFF_RATE_MID;
  }

  com_pub.publish(com);
}

