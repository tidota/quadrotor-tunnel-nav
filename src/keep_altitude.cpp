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
  vel_pub = n.advertise<geometry_msgs::Twist>("keep_altitude", 1);
  vel_sub = n.subscribe("turn", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Keep_Alt::~Keep_Alt()
{
  this->stop();
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
  boost::mutex::scoped_lock lock(vel_mutex);

  // input check
  if(rng_u[1].range - rng_d[1].range > DIST_OFF_ALT || rng_d[1].range - rng_u[1].range > DIST_OFF_ALT)
  {
    ROS_INFO("KEEP THE ALTITUDE");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.linear.z = VEL_ALT * (rng_d[1].range < rng_u[1].range)? 1: -1;
    //if(rng_h[2] > rng_h[0])
      //vel.linear.x = VEL_STRAIGHT;
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Keep_Alt::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

