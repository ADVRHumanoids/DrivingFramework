#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_PRESSURE_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CENTER_OF_PRESSURE_H

#include "mwoibn/robot_class/contacts.h"
#include "mwoibn/robot_points/point.h"
#include "mwoibn/point_handling/raw_positions_handler.h"

namespace mwoibn
{
namespace robot_points
{

class CenterOfPressure: public Point
{

public:
  CenterOfPressure(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, mwoibn::robot_class::Contacts& contacts);

  virtual ~CenterOfPressure() {}

  virtual void compute();

  virtual void update(bool jacobian = true);

protected:
  mwoibn::robot_class::Contacts& _contacts;
  mwoibn::VectorN _forces;
  std::vector<mwoibn::Matrix3> _null_space;
  mwoibn::Vector3 _temp;
  point_handling::RawPositionsHandler _points;
  double _sum_force;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
