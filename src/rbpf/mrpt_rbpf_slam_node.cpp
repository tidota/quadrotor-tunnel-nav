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

  std::ofstream f;
  double currT = ros::Time::now().toSec();
  f.open("eval_results.csv", std::ofstream::out);
  if (f)
  {
    f << "Time,Odometry Err, SLAM Err" << std::endl;
  }
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    slam.procSensoryData();
    
    if (f)
    {
      double tempT = slam.getCurrentTime();
      if (currT != tempT)
      {
        currT = tempT;
        f << currT << ",";
        f << slam.getOdomErr() << ",";
        f << slam.getSLAMErr() << std::endl;
      }
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
