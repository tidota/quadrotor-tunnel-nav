// middle_line.cpp

#include "wall_follow/Right_Wall.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "right_wall");

  Right_Wall* obj = new Right_Wall();

  ros::spin();

  delete obj;

  return(0);
}
