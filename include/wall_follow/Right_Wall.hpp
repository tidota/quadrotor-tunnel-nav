#ifndef _RIGHT_WALL_HPP
#define _RIGHT_WALL_HPP

#include "reactive/Middle_Line.hpp"
#include "wall_follow/common.hpp"

// Follow the right wall without caring about the left side
#define DIST_RIGHT_FOLLOW DIST_WALL
#define DIST_OFF_RATE_RIGHT_FOLLOW 0.01
#define MAX_VEL_RIGHT_FOLLOW 3.0
class Right_Wall: public Middle_Line
{
public:
  Right_Wall();
private:
  void command() override;
};

#endif
