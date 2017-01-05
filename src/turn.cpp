// turn.cpp

#include "layers.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "turn");

  Turn* obj = new Turn();

  ros::spin();

  delete obj;

  return(0);
}

// ============================================================================================
// Constructor
// ============================================================================================
Turn::Turn()
{
  // set up for publisher, subscriber
  ros::NodeHandle n;
  vel_pub = n.advertise<geometry_msgs::Twist>("turn", 1);
  vel_sub = n.subscribe("adjust_direction", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Turn::~Turn()
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
void Turn::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);

  // input check
  if(rng_h[6].range > rng_h[0].range && rng_h[7].range > rng_h[6].range * sqrt(2) * DIST_RATE_TURN)
  {
    ROS_INFO("TURN RIGHT");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.angular.z = -VEL_TURN;
  }
  else if(rng_h[6].range > rng_h[0].range && rng_h[7].range <= rng_h[6].range * sqrt(2) * DIST_RATE_TURN)
  {
    ROS_INFO("TURN LEFT");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.angular.z = VEL_TURN;
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Turn::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

