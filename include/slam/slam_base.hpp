// slam_base.hpp
// 190113
// Base class and macros to use ranging sensors.
//

#ifndef _SLAM_BASE_HPP
#define _SLAM_BASE_HPP

#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

///////////////////////////////////////////////////////////////
// Constants
#define TIME_INT 0.05

#define TOPIC_RANGE_H0 "range_front"
#define TOPIC_RANGE_H1 "range_lfront"
#define TOPIC_RANGE_H2 "range_left"
#define TOPIC_RANGE_H3 "range_lrear"
#define TOPIC_RANGE_H4 "range_rear"
#define TOPIC_RANGE_H5 "range_rrear"
#define TOPIC_RANGE_H6 "range_right"
#define TOPIC_RANGE_H7 "range_rfront"
#define TOPIC_RANGE_U0 "range_ufront"
#define TOPIC_RANGE_U1 "range_up"
#define TOPIC_RANGE_U2 "range_urear"
#define TOPIC_RANGE_D0 "range_dfront"
#define TOPIC_RANGE_D1 "range_down"
#define TOPIC_RANGE_D2 "range_drear"

#define PI 3.14159265

//////////////////////////////////////////////////////////////////
// macros
#define def_updateRange_H(_NUM_) void updateRange_H##_NUM_(\
  const sensor_msgs::Range::ConstPtr& new_range)\
{\
  rng_h[_NUM_] = *new_range;\
}

#define def_updateRange_U(_NUM_) void updateRange_U##_NUM_(\
  const sensor_msgs::Range::ConstPtr& new_range)\
{\
  rng_u[_NUM_] = *new_range;\
}

#define def_updateRange_D(_NUM_) void updateRange_D##_NUM_(\
  const sensor_msgs::Range::ConstPtr& new_range)\
{\
  rng_d[_NUM_] = *new_range;\
}

#define subscribe_H(_NUM_) \
  rng_h_sub[_NUM_] = n.subscribe(\
    TOPIC_RANGE_H##_NUM_, 1, &SLAM_BASE::updateRange_H##_NUM_, this);
#define subscribe_U(_NUM_) \
  rng_u_sub[_NUM_] = n.subscribe(\
    TOPIC_RANGE_U##_NUM_, 1, &SLAM_BASE::updateRange_U##_NUM_, this);
#define subscribe_D(_NUM_) \
  rng_d_sub[_NUM_] = n.subscribe(\
    TOPIC_RANGE_D##_NUM_, 1, &SLAM_BASE::updateRange_D##_NUM_, this);

// =============================================================================
// SLAM_BASE class
// it contains sensory data necessary used for SLAM
// =============================================================================
class SLAM_BASE
{
public:
  SLAM_BASE()
  {
    // set up for timer and subscribers
    ros::NodeHandle n;
    timer = n.createTimer(
      ros::Duration(TIME_INT), boost::bind(&SLAM_BASE::proc, this));

    subscribe_H(0)
    subscribe_H(1)
    subscribe_H(2)
    subscribe_H(3)
    subscribe_H(4)
    subscribe_H(5)
    subscribe_H(6)
    subscribe_H(7)
    subscribe_U(0)
    subscribe_U(1)
    subscribe_U(2)
    subscribe_D(0)
    subscribe_D(1)
    subscribe_D(2)

    r_pose_sub = n.subscribe(
      "ground_truth_to_tf/pose", 1, &SLAM_BASE::updateRobotPose, this);

    map_pub = n.advertise<octomap_msgs::Octomap>("octomap", 1);

    // sensor's pose w.r.t. the robot's reference frame.
    pose_h[0] = octomath::Pose6D( 0.05,  0.00,  0.05,  0.00,  0.00,  0.00);
    pose_h[1] = octomath::Pose6D( 0.05,  0.05,  0.05,  0.00,  0.00,  PI/4);
    pose_h[2] = octomath::Pose6D( 0.00,  0.05,  0.05,  0.00,  0.00,  PI/2);
    pose_h[3] = octomath::Pose6D(-0.05,  0.05,  0.05,  0.00,  0.00,3*PI/4);
    pose_h[4] = octomath::Pose6D(-0.05,  0.00,  0.05,  0.00,  0.00,  PI);
    pose_h[5] = octomath::Pose6D(-0.05, -0.05,  0.05,  0.00,  0.00,5*PI/4);
    pose_h[6] = octomath::Pose6D( 0.00, -0.05,  0.05,  0.00,  0.00,3*PI/2);
    pose_h[7] = octomath::Pose6D( 0.05, -0.05,  0.05,  0.00,  0.00,7*PI/4);
    pose_d[0] = octomath::Pose6D( 0.05/3,  0.00,  0.05,  0.00,  PI/4,  0.00);
    pose_d[1] = octomath::Pose6D( 0.0000,  0.00,  0.05,  0.00,  PI/2,  0.00);
    pose_d[2] = octomath::Pose6D(-0.05/3,  0.00,  0.05,  0.00,  PI/4,  PI);
    pose_u[0] = octomath::Pose6D( 0.05/3,  0.00, -0.05,  0.00, -PI/4,  0.00);
    pose_u[1] = octomath::Pose6D( 0.0000,  0.00, -0.05,  0.00, -PI/2,  0.00);
    pose_u[2] = octomath::Pose6D(-0.05/3,  0.00, -0.05,  0.00, -PI/4,  PI);

    m_octree = new octomap::OcTree(0.25);
    m_octree->setProbHit(0.7);
    m_octree->setProbMiss(0.4);
    m_octree->setClampingThresMin(0.12);
    m_octree->setClampingThresMax(0.97);

    occupiedNodesVis.markers.resize(m_octree->getTreeDepth()+1);

    marker_pub = n.advertise<visualization_msgs::MarkerArray>("map_marker", 1);
    m_color.r = 1;
    m_color.g = 1;
    m_color.b = 0.3;
    m_color.a = 1;
    marker_counter = 0;
  }
  virtual ~SLAM_BASE()
  {
    delete m_octree;
  }

protected:
  virtual void proc() = 0;
  ros::Timer timer;

private:
  ros::Subscriber rng_h_sub[8];
  ros::Subscriber rng_u_sub[3];
  ros::Subscriber rng_d_sub[3];

  ros::Subscriber r_pose_sub;

protected:
  sensor_msgs::Range rng_h[8];
  sensor_msgs::Range rng_u[3];
  sensor_msgs::Range rng_d[3];

  octomath::Pose6D pose_h[8];
  octomath::Pose6D pose_u[3];
  octomath::Pose6D pose_d[3];

  octomap::OcTree *m_octree;

  geometry_msgs::PoseStamped r_pose;

  ros::Publisher map_pub;

  int marker_counter;
  visualization_msgs::MarkerArray occupiedNodesVis;
  std_msgs::ColorRGBA m_color;
  ros::Publisher marker_pub;

public:
  def_updateRange_H(0)
  def_updateRange_H(1)
  def_updateRange_H(2)
  def_updateRange_H(3)
  def_updateRange_H(4)
  def_updateRange_H(5)
  def_updateRange_H(6)
  def_updateRange_H(7)
  def_updateRange_U(0)
  def_updateRange_U(1)
  def_updateRange_U(2)
  def_updateRange_D(0)
  def_updateRange_D(1)
  def_updateRange_D(2)

  void updateRobotPose(const geometry_msgs::PoseStamped::ConstPtr& new_pose)
  {
    ROS_INFO("This message is from updateRobotPose");
    r_pose = *new_pose;
  }
};

#endif // _SLAM_BASE_HPP
