#ifndef __MWOIBN__ROBOT_CLASS__FRAME_ORIENTATION_H
#define __MWOIBN__ROBOT_CLASS__FRAME_ORIENTATION_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/frame.h"


namespace mwoibn
{
namespace robot_points
{

class FrameOrientation: public State
{

public:
  template<typename Body>
  FrameOrientation(Body body, mwoibn::robot_class::Robot& robot): State(robot.getModel(), robot.state), point(body, robot.getModel(), robot.state){
    _point.setZero(3);
  }

  using Point::operator=;


  void update(bool jacobian) {
      //compute();
      if(jacobian)
        computeJacobian();
    }

  void compute(){ mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);}

  void computeJacobian() {_jacobian.noalias() = point.getOrientationJacobian();}

  mwoibn::point_handling::Frame point;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
