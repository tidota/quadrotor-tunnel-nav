// go_straight.cpp

#include <cstdlib>

#include "wall_follow/Keep_Going.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "keep_going");

  Keep_Going* obj;
  if (argc == 2)
    obj = new Keep_Going(std::atof(argv[1]));
  else
    obj = new Keep_Going(2.0);


  ros::spin();

  delete obj;

  return(0);
}
