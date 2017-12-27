// go_straight.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_straight");

  Go_Straight* obj = new Go_Straight();

  ros::spin();

  delete obj;

  return(0);
}

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

  //com = list_com[TOPIC_STR];

  // input check
  com.message = "GO STRAIGHT";
  // calculate the output
  if(rng_h[0].range <= DIST_MAX)
    com.vel.linear.x = MAX_VEL_STRAIGHT * rng_h[0].range / DIST_MAX;
  else
    com.vel.linear.x = MAX_VEL_STRAIGHT;

  com_pub.publish(com);
}

