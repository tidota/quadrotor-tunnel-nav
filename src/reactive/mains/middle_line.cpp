// middle_line.cpp

#include "reactive/Middle_Line.hpp"

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
