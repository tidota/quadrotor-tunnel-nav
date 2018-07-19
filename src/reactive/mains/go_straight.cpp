// go_straight.cpp

#include "reactive/Go_Straight.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "go_straight");

  Go_Straight* obj = new Go_Straight();

  ros::spin();

  delete obj;

  return(0);
}
