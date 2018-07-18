#ifndef _KEEP_GOING_HPP
#define _KEEP_GOING_HPP

#include "reactive/layers.hpp"
#include "reactive/Go_Straight.hpp"

// Go straight at the constant speed
class Keep_Going: public Go_Straight
{
public:
  Keep_Going();
private:
  void command() override;
};

#endif
