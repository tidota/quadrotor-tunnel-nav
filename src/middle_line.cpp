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
  vel_pub = n.advertise<geometry_msgs::Twist>("middle_line", 1);
  vel_sub = n.subscribe("go_straight", 1, &LAYER_BASE::updateVel, (LAYER_BASE*)this);
}
Middle_Line::~Middle_Line()
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
void Middle_Line::command()
{
  boost::mutex::scoped_lock lock(vel_mutex);

  // input check
  if(rng_h[2].range <= rng_h[0].range &&
  (rng_h[6].range < rng_h[2].range - DIST_OFF || rng_h[6].range > rng_h[2].range + DIST_OFF))
  {
    ROS_INFO("STAY ON THE MIDDLE LINE");
    vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
    vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
    // calculate the output
    vel.linear.y = (rng_h[6].range < rng_h[2].range)? VEL_MIDDLE: -VEL_MIDDLE;
  }

  vel_pub.publish(vel);
}

// ============================================================================================
// stop
//
// it stops the UAV so that the machine stays on the current location.
// ============================================================================================
void Middle_Line::stop()
{
  boost::mutex::scoped_lock lock(vel_mutex);
  vel.linear.x = 0; vel.linear.y = 0; vel.linear.z = 0;
  vel.angular.x = 0; vel.angular.y = 0; vel.angular.z = 0;
  vel_pub.publish(vel);
  timer.stop();
}

