#ifndef __MWOIBN__ROBOT_CLASS__CUSTOM_FRAME_H
#define __MWOIBN__ROBOT_CLASS__CUSTOM_FRAME_H

#include "mwoibn/robot_points/point.h"
#include "mwoibn/point_handling/rotation.h"// ?


namespace mwoibn
{
namespace robot_points
{

template<typename RigidPoint>
class CustomFrame: public Point
{

public:
  CustomFrame(RigidPoint rigid_point, mwoibn::robot_class::Robot& robot):
    Point(rigid_point.size(), rigid_point.dofs()),
    rigid_point(rigid_point), rotation(this->rigid_point){}

  using Point::operator=;

  virtual ~CustomFrame() {}

  void update(bool jacobian = true) {
      compute();
    }

  void compute(){
      rigid_point.setFixed( rotation.getFixed()*_point);
  }

  void computeJacobian() {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__); // or empty function
  } // makes no sense


    RigidPoint rigid_point;
    mwoibn::point_handling::Rotation rotation;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
