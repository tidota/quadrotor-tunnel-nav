#ifndef _MIDDLE_LINE_HPP
#define _MIDDLE_LINE_HPP

#include "reactive/layers.hpp"

class Middle_Line: public LAYER_BASE
{
public:
  Middle_Line();
private:
  virtual void command();
};

#endif
