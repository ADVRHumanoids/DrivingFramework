#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_MASS_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_MASS_H

#include <rbdl/rbdl.h>
#include "mwoibn/point_handling/raw_positions_handler.h"
#include <memory>
#include "mwoibn/robot_points/state.h"

namespace mwoibn
{
namespace robot_points
{
class CenterOfMass: public State
{

public:
  CenterOfMass(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state);

  virtual ~CenterOfMass() {}
  using Point::operator=;

  virtual void compute();

  virtual void computeJacobian();

  virtual void computeDerivative();

  virtual void accelerationComponent();

  const mwoibn::Vector3& derivative(){return _d_com;}
  const mwoibn::Vector3& acceleration(){return _dd_com;}

  double mass(){return _mass;}

protected:
  point_handling::RawPositionsHandler _points;
  std::vector<double> _masses;
  double _mass = 0;
  mwoibn::Vector3 _com, _d_com, _dd_com;
};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
