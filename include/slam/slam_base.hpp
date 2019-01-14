// slam_base.hpp
// 190113
// Base class and macros to use ranging sensors.
//

#ifndef _SLAM_BASE_HPP
#define _SLAM_BASE_HPP

#include <map>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

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
  }
  virtual ~SLAM_BASE()
  {
  }

protected:
  virtual void proc() = 0;
  ros::Timer timer;

private:
  ros::Subscriber rng_h_sub[8];
  ros::Subscriber rng_u_sub[3];
  ros::Subscriber rng_d_sub[3];

protected:
  sensor_msgs::Range rng_h[8];
  sensor_msgs::Range rng_u[3];
  sensor_msgs::Range rng_d[3];

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
};

#endif // _SLAM_BASE_HPP
