#ifndef _KEEP_GOING_HPP
#define _KEEP_GOING_HPP

#include "reactive/Go_Straight.hpp"
#include "wall_follow/common.hpp"

// Go straight at the constant speed
class Keep_Going: public Go_Straight
{
public:
  Keep_Going();
private:
  void command() override;
};

#endif
