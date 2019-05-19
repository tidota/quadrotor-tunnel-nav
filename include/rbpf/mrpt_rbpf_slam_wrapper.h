#pragma once

#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include "rbpf/mrpt_rbpf_slam.h"

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensor_msgs/Range.h>

// mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>

#include <mrpt_bridge/range.h>

#include <mrpt_bridge/time.h>

// map visualization
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#define PI 3.14159265

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslamWrapper class provides  the ROS wrapper for Rao-Blackwellized Particle filter SLAM from MRPT
 *libraries.
 *
 */
class PFslamWrapper : public PFslam
{
public:
  PFslamWrapper();
  ~PFslamWrapper() = default;

  /**
   * @brief Read the parameters from launch file
   */
  bool getParams(const ros::NodeHandle& nh_p);

  /**
   * @brief Initialize publishers subscribers and RBPF slam
   */
  bool init(ros::NodeHandle& nh);

  /**
   * @brief Play rawlog file
   *
   * @return true if rawlog file exists and played
   */
  //bool rawlogPlay();

  /**
   * @brief Publish beacon or grid map and robot pose
   */
  void publishMapPose();

  /**
   * @brief Callback function for the ranging sensors
   *
   * Given the ranging data  wait for odometry,
   * create the pair of action and observation,
   * implement one SLAM update,
   * publish map and pose.
   *
   * @param msg  the range message
   */
  void rangeCallback(const sensor_msgs::Range& msg);

  /**
   * @brief Process on the received sensory data.
   */
  void procSensoryData();

  /**
   * @brief Wait for transform between odometry frame and the robot frame
   *
   * @param[out] des position of the robot with respect to odometry frame
   * @param[in]  target_frame the odometry tf frame
   * @param[in]  source_frame the robot tf frame
   * @param[in]  time timestamp of the observation for which we want to retrieve the position of the robot
   * @param[in]  timeout timeout for odometry waiting
   * @param[in]  polling_sleep_duration timeout for transform wait
   *
   * @return true if there is transform from odometry to the robot
   */
  bool waitForTransform(mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
                        const ros::Time& time, const ros::Duration& timeout,
                        const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

  /**
   * @brief Update the pose of the sensor with respect to the robot
   *
   *@param frame_id the frame of the sensors
   */
  void updateSensorPose(const std::string& frame_id);

  /**
   * @brief Publish tf tree
   *
   */
  void publishTF();

  /**
   * @brief Publish map (for visualization)
   *
   */
  void publishVisMap();

private:
  std::string ini_filename_;     ///< name of ini file
  std::string gtruthw_frame_id_;  ///< /ground truth world
  std::string gtruthb_frame_id_;  ///< /ground truth base_link
  std::string global_frame_id_;  ///< /map frame
  std::string odom_frame_id_;    ///< /odom frame
  std::string base_frame_id_;    ///< robot frame

  // maximum frequency of updates
  double rate_;

  // Sensor source
  std::string sensor_source_;
  std::map<std::string, mrpt::poses::CPose3D> range_poses_;   ///< range poses with respect to the map

  // buffer of sensor messages
  std::map<std::string, std::shared_ptr<sensor_msgs::Range> > sensor_buffer;

  // Subscribers
  std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics

  ros::Publisher pub_map_, /*pub_metadata_,*/ pub_particles_;

  tf::TransformListener listenerTF_;         ///< transform listener
  tf::TransformBroadcaster tf_broadcaster_;  ///< transform broadcaster
#if MRPT_VERSION >= 0x199
  mrpt::system::CTicTac tictac_;  ///< timer for SLAM performance evaluation
#else
  mrpt::utils::CTicTac tictac_;
#endif
  float t_exec_;  ///< the time taken for one SLAM update execution

  // visualization of the map
  std_msgs::ColorRGBA m_color_occupied;
  ros::Publisher marker_occupied_pub;
  int marker_counter;

  std::mutex sensor_mutex; /// mutex for sensor acquisition

// for evaluation
private:
  mrpt::poses::CPose3D odometry_; /// location estimated by odometry
  mrpt::poses::CPose3D odometryOrg_; /// original location of odometry_
  bool odometryOrgUnitialized_; /// whether odometryOrg_ is uninitialized
  mrpt::poses::CPose3D location_; /// location estimated by SLAM
  mrpt::poses::CPose3D locationOrg_; /// original location of location_
  bool locationOrgUnitialized_; /// whether locationOrg_ is uninitialized
  mrpt::poses::CPose3D gtruthLoc_; /// ground truth location
  mrpt::poses::CPose3D gtruthLocOrg_; /// original location of gtruthLoc_
  bool gtruthLocOrgUnitialized_; /// whether gtruthLocOrg_ is uninitialized
  ros::Time currentTime_;

public:
  double getCurrentTime()
  {
    return currentTime_.toSec();
  }
  double getOdomErr()
  {
    return ((odometry_ - odometryOrg_) - (gtruthLoc_ - gtruthLocOrg_)).norm();
  }
  double getSLAMErr()
  {
    return ((location_ - locationOrg_) - (gtruthLoc_ - gtruthLocOrg_)).norm();
  }
};
}  // namespace mrpt_rbpf_slam
