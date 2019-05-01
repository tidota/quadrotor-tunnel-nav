#include <iostream>

#include "rbpf/mrpt_rbpf_slam_wrapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mrpt_rpbf_slam");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  // Setup ros loop frequency from params
  double frequency;
  nh_p.param("update_loop_frequency", frequency, 100.);
  ros::Rate rate(frequency);

  mrpt_rbpf_slam::PFslamWrapper slam;
  // Read parameters and configure node
  // and setup callbacks
  if (!slam.getParams(nh_p) || !slam.init(nh))
  {
    return EXIT_FAILURE;
  }

  ros::Duration(1).sleep();

/* to remove
  // If play from rawlog file options is specified
  // play and then terminate application
  if (slam.rawlogPlay())
  {
    return EXIT_SUCCESS;
  }
*/

  std::ofstream f;
  f.open("eval_results.csv", std::ofstream::out);
  if (f)
  {
    f << "Time,Odometry Err, SLAM Err" << std::endl;
  }
  // Otherwise work as a usual rosnode
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (f)
    {
      f << slam.getCurrentTime() << ",";
      f << slam.getOdomErr() << ",";
      f << slam.getSLAMErr() << std::endl;
      ROS_INFO_STREAM("Time diff: " << (slam.getCurrentTime() - ros::Time::now().toSec()));
    }
    else
    {
      ROS_INFO_STREAM("FILE NOT OPEN, cannot write results!!!!");
    }
  }
  if (f)
  {
    f.close();
  }
}
