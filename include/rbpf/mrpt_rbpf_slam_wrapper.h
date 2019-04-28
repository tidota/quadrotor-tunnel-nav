#pragma once

#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <map>
#include <memory>
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
   * @brief Get the odometry for received observation
   *
   * @param[out] odometry odometry for received observation
   * @param[in]  msg_header timestamp of the observation
   */
  void odometryForCallback(mrpt::poses::CPose3D& odometry, const std_msgs::Header& msg_header);

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
  std::string global_frame_id_;  ///< /map frame
  std::string odom_frame_id_;    ///< /odom frame
  std::string base_frame_id_;    ///< robot frame

  // maximum frequency of updates
  double freq_;

  // Sensor source
  std::string sensor_source_;
  std::map<std::string, mrpt::poses::CPose3D> range_poses_;   ///< range poses with respect to the map

  // buffer of sensor messages
  std::map<std::string, std::queue< std::shared_ptr<sensor_msgs::Range> > > sensor_buffer;

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
  float t_exec_;  ///< the time which take one SLAM update execution

  // visualization of the map
  std_msgs::ColorRGBA m_color_occupied;
  ros::Publisher marker_occupied_pub;
  int marker_counter;
};
}  // namespace mrpt_rbpf_slam
