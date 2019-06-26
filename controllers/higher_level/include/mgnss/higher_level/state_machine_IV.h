#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_IV_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_IV_H

#include "mgnss/higher_level/state_machine.h"



namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class StateMachineIV: public StateMachine
{

public:
  StateMachineIV(mwoibn::robot_class::Robot& robot, YAML::Node config);

  virtual void update();
  StateTransformation state_I, state_II;

protected:
  virtual void _marginJacobians();
  virtual void _workspaceJacobian();
  std::unique_ptr<mwoibn::robot_points::LinearPoint> _pelvis;

};
}
}
#endif
