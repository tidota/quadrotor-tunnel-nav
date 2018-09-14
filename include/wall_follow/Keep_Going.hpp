#ifndef _KEEP_GOING_HPP
#define _KEEP_GOING_HPP

#include <mutex>

#include <std_msgs/Float32.h>

#include "reactive/Go_Straight.hpp"
#include "wall_follow/common.hpp"

// Go straight at the constant speed
class Keep_Going: public Go_Straight
{
public:
  Keep_Going(const double _vel);
  void update_velocity(const std_msgs::Float32::ConstPtr& new_vel);
private:
  void command() override;

  ros::Subscriber vel_change_sub;
  double vel;
};

#endif
