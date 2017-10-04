#ifndef ROBOT_CLASS_CENTER_OF_MASS_H
#define ROBOT_CLASS_CENTER_OF_MASS_H

#include <rbdl/rbdl.h>
#include "mwoibn/point_handling/raw_positions_handler.h"
#include <memory>
#include "mwoibn/robot_class/robot_class.h"

namespace mwoibn
{
namespace robot_class
{

class CenterOfMass
{

public:
  CenterOfMass(RigidBodyDynamics::Model& model,
               const mwoibn::VectorN& positions,
               const mwoibn::VectorN& velocities);
  virtual ~CenterOfMass() {}

  void compute();

  void computeJacobian();

  void update(bool jacobian = true);

  mwoibn::Matrix getJacobian() const { return _jacobian; }
  mwoibn::Vector3 get() const { return _com; }

protected:
  point_handling::RawPositionsHandler _points;
  std::vector<double> _masses;
  double _mass = 0;
  mwoibn::Vector3 _com;
  mwoibn::Matrix _jacobian;
  RigidBodyDynamics::Model& _model;

  const mwoibn::VectorN& _positions;
  const mwoibn::VectorN& _velocities;
};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
