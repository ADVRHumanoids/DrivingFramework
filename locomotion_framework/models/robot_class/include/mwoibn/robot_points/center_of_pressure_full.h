#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_PRESSURE_FAST_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_PRESSURE_FAST_H

#include "mwoibn/robot_points/centre_of_pressure.h"

namespace mwoibn

namespace robot_points
{

class CenterOfPressureFull: public CenterOfPressure
{

public:
  CenterOfPressureFull(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, mwoibn::robot_class::Contacts contacts);

  virtual ~CenterOfPressureFull() {}

  virtual void computeJacobian();

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
