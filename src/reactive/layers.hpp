// layers.hpp
// 170104
// basic definitions and class definitions
// 

#ifndef _LAYERS_HPP
#define _LAYERS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_tunnel_nav/Com.h>
#include <sensor_msgs/Range.h>
#include "boost/thread/mutex.hpp"

#include <signal.h>

///////////////////////////////////////////////////////////////
// Constants
#define TIME_INT 0.05

#define DIST_OBS 0.5
#define VEL_OBS 0.5
#define DIST_OFF_ALT 0.5
#define VEL_ALT 0.3

#define DIST_RATE_TURN 1.1
#define VEL_TURN 1.0

#define DIST_RATE_ADJL 0.9
#define DIST_RATE_ADJR 1.1

#define DIST_MAX 10.0
#define DIST_OFF 1.0
#define VEL_MIDDLE 0.5

#define VEL_STRAIGHT 0.8
#define VEL_FIND 1.0


#define TOPIC_RANGE_H0 "/range_front"
#define TOPIC_RANGE_H1 "/range_lfront"
#define TOPIC_RANGE_H2 "/range_left"
#define TOPIC_RANGE_H3 "/range_lrear"
#define TOPIC_RANGE_H4 "/range_rear"
#define TOPIC_RANGE_H5 "/range_rrear"
#define TOPIC_RANGE_H6 "/range_right"
#define TOPIC_RANGE_H7 "/range_rfront"
#define TOPIC_RANGE_U0 "/range_ufront"
#define TOPIC_RANGE_U1 "/range_up"
#define TOPIC_RANGE_U2 "/range_urear"
#define TOPIC_RANGE_D0 "/range_dfront"
#define TOPIC_RANGE_D1 "/range_down"
#define TOPIC_RANGE_D2 "/range_drear"

//////////////////////////////////////////////////////////////////
// macros
#define def_updateRange_H(_NUM_) void updateRange_H##_NUM_(const sensor_msgs::Range::ConstPtr& new_range)\
  {\
    boost::mutex::scoped_lock lock(rng_h_mutex[_NUM_]);\
    rng_h[_NUM_] = *new_range;\
  }

#define def_updateRange_U(_NUM_) void updateRange_U##_NUM_(const sensor_msgs::Range::ConstPtr& new_range)\
  {\
    boost::mutex::scoped_lock lock(rng_u_mutex[_NUM_]);\
    rng_u[_NUM_] = *new_range;\
  }

#define def_updateRange_D(_NUM_) void updateRange_D##_NUM_(const sensor_msgs::Range::ConstPtr& new_range)\
  {\
    boost::mutex::scoped_lock lock(rng_d_mutex[_NUM_]);\
    rng_d[_NUM_] = *new_range;\
  }

#define subscribe_H(_NUM_) rng_h_sub[_NUM_] = n.subscribe(TOPIC_RANGE_H##_NUM_, 1, &LAYER_BASE::updateRange_H##_NUM_, this);
#define subscribe_U(_NUM_) rng_u_sub[_NUM_] = n.subscribe(TOPIC_RANGE_U##_NUM_, 1, &LAYER_BASE::updateRange_U##_NUM_, this);
#define subscribe_D(_NUM_) rng_d_sub[_NUM_] = n.subscribe(TOPIC_RANGE_D##_NUM_, 1, &LAYER_BASE::updateRange_D##_NUM_, this);

// ============================================================================================
// LAYER_BASE class
// it contains sensory data necessary for control
// ============================================================================================
class LAYER_BASE
{
public:
  LAYER_BASE()
  {
    // set up for timer and subscribers
    ros::NodeHandle n;
    timer = n.createTimer(ros::Duration(TIME_INT), boost::bind(&LAYER_BASE::command, this));

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
  ~LAYER_BASE()
  {
    timer.stop();
  }

protected:
  virtual void command() = 0;

  ros::Publisher com_pub;
  ros::Subscriber com_sub;

  boost::mutex com_mutex;
  quadrotor_tunnel_nav::Com com;

  ros::Timer timer;

public:
  void updateCom(const quadrotor_tunnel_nav::Com::ConstPtr& new_com)
  {
    boost::mutex::scoped_lock lock(com_mutex);
    com = *new_com;
  }

private:

  ros::Subscriber rng_h_sub[8];
  ros::Subscriber rng_u_sub[3];
  ros::Subscriber rng_d_sub[3];

protected:
  sensor_msgs::Range rng_h[8];
  sensor_msgs::Range rng_u[3];
  sensor_msgs::Range rng_d[3];

  boost::mutex rng_h_mutex[8];
  boost::mutex rng_u_mutex[3];
  boost::mutex rng_d_mutex[3];

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

// ============================================================================================
// UAV_Control class
// it contains everything necessary for control
// ============================================================================================
class UAV_Control : public LAYER_BASE
{
public:
  // the instance of this class must be single
  // so this function must be called to create the object.
  static UAV_Control *create_control();

  // to release the memory, call this function.
  static void kill_control();

protected:
  static ros::Publisher vel_pub;

private:
  UAV_Control();

  void command();

  static void quit(int);

  static UAV_Control *p_control;

};

// ============================================================================================
// LAYER classes
// ============================================================================================
class Obs_Avoid: public LAYER_BASE
{
public:
  Obs_Avoid();
private:
  void command();
};
class Keep_Alt: public LAYER_BASE
{
public:
  Keep_Alt();
private:
  void command();
};
class Turn: public LAYER_BASE
{
public:
  Turn();
private:
  void command();
};
class Steer: public LAYER_BASE
{
public:
  Steer();
private:
  void command();
};
class Middle_Line: public LAYER_BASE
{
public:
  Middle_Line();
private:
  void command();
};
class Go_Straight: public LAYER_BASE
{
public:
  Go_Straight();
private:
  void command();
};
class Find_Wall: public LAYER_BASE
{
public:
  Find_Wall();
private:
  void command();
};

#endif // _LAYERS_HPP
