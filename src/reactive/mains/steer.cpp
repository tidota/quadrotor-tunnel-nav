// steer.cpp

#include "reactive/Steer.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "steer");

  Steer* obj = new Steer();

  ros::spin();

  delete obj;

  return(0);
}
