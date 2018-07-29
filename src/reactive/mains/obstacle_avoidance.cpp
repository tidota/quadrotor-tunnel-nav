// obstacle_avoidance.cpp

#include "reactive/Obs_Avoid.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoidance");

  Obs_Avoid* obj = new Obs_Avoid();

  ros::spin();

  delete obj;

  return(0);
}
