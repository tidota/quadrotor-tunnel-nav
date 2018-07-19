// turn.cpp

#include "reactive/Turn.hpp"

// ============================================================================================
// Constructor
// ============================================================================================
Turn::Turn()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  com_pub = n.advertise<quadrotor_tunnel_nav::Com>(TOPIC_TRN, 1);
  list_com_sub[TOPIC_GO] = n.subscribe(TOPIC_GO, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
  list_com_sub[TOPIC_STR] = n.subscribe(TOPIC_STR, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
  list_com_sub[TOPIC_MID] = n.subscribe(TOPIC_MID, 1, &LAYER_BASE::updateCom, (LAYER_BASE*)this);
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Turn::command()
{
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com1 = list_com[TOPIC_GO];
  quadrotor_tunnel_nav::Com com2 = list_com[TOPIC_STR];
  quadrotor_tunnel_nav::Com com3 = list_com[TOPIC_MID];
  quadrotor_tunnel_nav::Com com = combCom(combCom(com1,com2),com3);

  double length_comp = fmax(rng_u[0].range, rng_d[0].range)/sqrt(2);

  // input check
  if(length_comp < DIST_MIN_TURN)
  {
    com.message = "TURN LEFT!";
    com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
    com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
    // calculate the output
    com.vel.angular.z = VEL_TURN;
  }
  else if(rng_u[0].range <= rng_u[1].range * sqrt(2) * DIST_RATE_TURN &&
          rng_d[0].range <= rng_d[1].range * sqrt(2) * DIST_RATE_TURN)
  {
    length_comp = fmax(rng_h[0].range, length_comp);

    if(rng_h[6].range > length_comp && rng_h[7].range > rng_h[6].range * sqrt(2) * DIST_RATE_TURN)
    {
      com.message = "TURN RIGHT";
      com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
      com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
      // calculate the output
      com.vel.angular.z = -VEL_TURN;
    }
    else if(rng_h[6].range > length_comp && rng_h[7].range <= rng_h[6].range * sqrt(2) * DIST_RATE_TURN)
    {
      com.message = "TURN LEFT";
      com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
      com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;
      // calculate the output
      com.vel.angular.z = VEL_TURN;
    }
  }

  com_pub.publish(com);
}
