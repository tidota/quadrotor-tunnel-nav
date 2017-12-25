// keep_altitude.cpp

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
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_ALT, 1);
  list_com_sub[TOPIC_TRN] = n.subscribe(TOPIC_TRN, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
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
  quadrotor_tunnel_nav::Com com;

  com = list_com[TOPIC_TRN];

  // diff_rate is the gap from the mid altitude with respect to z axis
  double mid_leng = (rng_u[1].range + rng_d[1].range)/2;
  double diff_leng = (rng_d[1].range - rng_u[1].range)/2;
  double diff_rate = (mid_leng != 0)? diff_leng/mid_leng: 0;

  // input check
  // if it is out of range defined by DIST_OFF_RATE_ALT, apply the max vel
  if(diff_rate < -DIST_OFF_RATE_ALT || DIST_OFF_RATE_ALT < diff_rate)
  {
    com.message = com.message + " + KEEP THE ALTITUDE";
    com.vel.linear.z += (diff_rate > DIST_OFF_RATE_ALT)? -MAX_VEL_ALT: MAX_VEL_ALT;
  }
  // if it is slightly off the mid altitude, apply a proportional value
  else if(diff_rate < -DIST_OFF_RATE_ALT*0.1 || DIST_OFF_RATE_ALT*0.1 < diff_rate)
  {
    com.message = com.message + " + KEEP THE ALTITUDE";
    com.vel.linear.z -= MAX_VEL_ALT * diff_rate / DIST_OFF_RATE_ALT;
  }

  com_pub.publish(com);
}

