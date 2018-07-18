// keep_altitude.cpp

#ifndef _KEEP_ALT_HPP
#define _KEEP_ALT_HPP

#include "reactive/layers.hpp"

class Keep_Alt: public LAYER_BASE
{
public:
  Keep_Alt();
private:
  virtual void command();
};

#endif
