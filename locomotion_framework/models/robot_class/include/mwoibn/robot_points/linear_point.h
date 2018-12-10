#ifndef __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H
#define __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/frame.h"


namespace mwoibn
{
namespace robot_points
{

class LinearPoint: public State
{

public:
  template<typename Body>
  LinearPoint(Body body, mwoibn::robot_class::Robot& robot): State(robot.getModel(), robot.state), point(body, robot.getModel(), robot.state){
    _point.setZero(3);
  }

  using Point::operator=;

  virtual ~LinearPoint() {}

  void compute(){_point.noalias() = point.getLinearWorld();}

  void computeJacobian() {_jacobian.noalias() = point.getPositionJacobian();}

  mwoibn::point_handling::Frame point;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
