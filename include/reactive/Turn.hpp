#ifndef _TURN_HPP
#define _TURN_HPP

#include "reactive/layers.hpp"

class Turn: public LAYER_BASE
{
public:
  Turn();
private:
  virtual void command();
};

#endif
