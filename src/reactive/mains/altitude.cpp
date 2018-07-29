#include "reactive/Keep_Alt.hpp"

// ============================================================================================
// main
// ============================================================================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "altitude");

  Keep_Alt* obj = new Keep_Alt();

  ros::spin();

  delete obj;

  return(0);
}
