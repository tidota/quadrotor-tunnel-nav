// uav_control.hpp
// 161230
// main portion of definitions
// 

#ifndef _UAV_CONTROL_HPP
#define _UAV_CONTROL_HPP

#include <signal.h>

#include "layer_base.hpp"

///////////////////////////////////////////////////////////////
// Constants

// ============================================================================================
// UAV_Control class
// it contains everything necessary for control
// ============================================================================================
class UAV_Control : public LAYER_BASE
{
public:
  // the instance of this class must be single
  // so this function must be called to create the object.
  static UAV_Control *create_control();

  // to release the memory, call this function.
  static void kill_control();

private:
  UAV_Control();

  void command();
  void stop();

  static void quit(int);

  static UAV_Control *p_control;

};

#endif // _UAV_CONTROL_HPP
