// go_straight.cpp

#include "wall_follow/Keep_Going.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "keep_going");

  Keep_Going* obj = new Keep_Going();

  ros::spin();

  delete obj;

  return(0);
}
