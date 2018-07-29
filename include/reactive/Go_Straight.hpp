#ifndef _GO_STRAIGHT_HPP
#define _GO_STRAIGHT_HPP

#include "reactive/layers.hpp"

class Go_Straight: public LAYER_BASE
{
public:
  Go_Straight();
private:
  virtual void command();
};

#endif
