#ifndef _OBS_AVOID_HPP
#define _OBS_AVOID_HPP

#include "reactive/layers.hpp"

class Obs_Avoid: public LAYER_BASE
{
public:
  Obs_Avoid();
private:
  virtual void command();
};

#endif
