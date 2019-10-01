#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_II_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_II_H

#include "mgnss/higher_level/state_machine.h"



namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class StateMachineII: public StateMachine
{

public:
  StateMachineII(mwoibn::robot_class::Robot& robot, YAML::Node config);

  virtual void update();
  StateTransformation state_I, state_II;

protected:
  virtual void _marginJacobians();
  virtual void _workspaceJacobian();
};
}
}
#endif
