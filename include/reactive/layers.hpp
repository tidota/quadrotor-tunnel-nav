// layers.hpp
// 170104
// basic definitions and class definitions
//

#ifndef _LAYERS_HPP
#define _LAYERS_HPP

#include <signal.h>

#include <map>

#include <geometry_msgs/Twist.h>
#include <quadrotor_tunnel_nav/Com.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_srvs/SetBool.h>

#include <boost/thread/mutex.hpp>


///////////////////////////////////////////////////////////////
// Constants
#define TIME_INT 0.05

#define DIST_OBS 0.5
#define VEL_OBS 1.5

#define DIST_OFF_RATE_ALT 0.01
#define MAX_VEL_ALT 3.0

#define DIST_MIN_TURN 1.0
#define DIST_RATE_TURN 1.1
#define VEL_TURN 3.0

#define DIST_MIN_STEER 1.5
#define DIST_RATE_STRL 0.99
#define DIST_RATE_STRR 1.01
#define MAX_VEL_STEER 3.0

#define DIST_MAX 10.0

#define DIST_MIN_MID 1.5
#define DIST_OFF_RATE_MID 0.01
#define MAX_VEL_MID 3.0

#define MAX_VEL_STRAIGHT 1.8
#define MIN_VEL_STRAIGHT 0.5

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


#define TOPIC_OBS "obs_avoid"
#define TOPIC_ALT "altitude"
#define TOPIC_TRN "turn"
#define TOPIC_GO  "straight"
#define TOPIC_STR "steer"
#define TOPIC_MID "middle"

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
  virtual ~LAYER_BASE()
  {
    timer.stop();
  }

protected:
  virtual void command() = 0;

  ros::Publisher com_pub;
  std::map<std::string,ros::Subscriber> list_com_sub;

  boost::mutex com_mutex;
  std::map<std::string,quadrotor_tunnel_nav::Com> list_com;

  ros::Timer timer;

  quadrotor_tunnel_nav::Com combCom(const quadrotor_tunnel_nav::Com& com1, const quadrotor_tunnel_nav::Com& com2)
  {
    quadrotor_tunnel_nav::Com com;
    com.message = com1.message + " | " + com2.message;
    com.vel.linear.x = com1.vel.linear.x + com2.vel.linear.x;
    com.vel.linear.y = com1.vel.linear.y + com2.vel.linear.y;
    com.vel.linear.z = com1.vel.linear.z + com2.vel.linear.z;
    com.vel.angular.x = com1.vel.angular.x + com2.vel.angular.x;
    com.vel.angular.y = com1.vel.angular.y + com2.vel.angular.y;
    com.vel.angular.z = com1.vel.angular.z + com2.vel.angular.z;

    return com;
  }


public:
  void updateCom(const ros::MessageEvent<quadrotor_tunnel_nav::Com const>& event)
  {
    const quadrotor_tunnel_nav::Com::ConstPtr& new_com = event.getMessage();
    const ros::M_string& header = event.getConnectionHeader();
    std::string topic = header.at("topic");

    boost::mutex::scoped_lock lock(com_mutex);
    list_com[topic.substr(topic.rfind("/") + 1)] = *new_com;
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

#endif // _LAYERS_HPP
