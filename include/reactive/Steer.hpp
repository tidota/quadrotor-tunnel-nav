#ifndef _STEER_HPP
#define _STEER_HPP

#include "reactive/layers.hpp"

class Steer: public LAYER_BASE
{
public:
  Steer();
private:
  virtual void command();
};

#endif
