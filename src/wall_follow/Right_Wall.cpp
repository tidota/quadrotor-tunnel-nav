#include "wall_follow/Right_Wall.hpp"

// ============================================================================================
// Constructor
// ============================================================================================
Right_Wall::Right_Wall(): Middle_Line()
{
}

// ============================================================================================
// command
//
// core part of control
// it controls the uav based on the received sensor data.
// it is to be called repeatedly by the timer.
// ============================================================================================
void Right_Wall::command()
{
  boost::mutex::scoped_lock lock(com_mutex);
  quadrotor_tunnel_nav::Com com;
  com.vel.linear.x = 0; com.vel.linear.y = 0; com.vel.linear.z = 0;
  com.vel.angular.x = 0; com.vel.angular.y = 0; com.vel.angular.z = 0;

  if(fmax(rng_u[0].range, rng_d[0].range)/sqrt(2) >= DIST_MIN_MID)
  {
    double lengR = (rng_h[5].range + rng_h[6].range + rng_h[7].range) / 3.0;

    double diff_rate = (lengR-DIST_RIGHT_FOLLOW)/DIST_RIGHT_FOLLOW;

    // input check
    // if the front side is clear and it is out of range from the middle line
    // apply a proportional value
    if(diff_rate < -DIST_OFF_RATE_RIGHT_FOLLOW || DIST_OFF_RATE_RIGHT_FOLLOW < diff_rate)
    {
      com.message = "STAY AT THE RIGHT SIDE";
      com.vel.linear.y -= MAX_VEL_RIGHT_FOLLOW * diff_rate;
    }
  }

  com_pub.publish(com);
}
