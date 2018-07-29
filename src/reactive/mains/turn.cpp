// turn.cpp

#include "reactive/Turn.hpp"

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
