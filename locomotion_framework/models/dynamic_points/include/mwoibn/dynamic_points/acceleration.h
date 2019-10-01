#ifndef __MWOIBN__DYNAMIC_POINTS__ACCELERATION_H
#define __MWOIBN__DYNAMIC_POINTS__ACCELERATION_H

#include "mwoibn/dynamic_points/dynamic_point.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace dynamic_points
{
template<typename AccelerationType, typename VelocityType>
class Acceleration: public DynamicPoint
{

  public:

    template<typename Body>
    Acceleration(Body body, mwoibn::robot_class::Robot& robot): DynamicPoint(robot.getModel(), robot.state),
                 _frame(body, robot.getModel(), robot.state)
    {
      _point.setZero(3);
      _acceleration.reset(new AccelerationType(_frame, "ZERO"));
      _velocity.reset(new VelocityType(_frame));
    }

    Acceleration(const Acceleration& other): DynamicPoint(other), _frame(other._frame){
      _point.setZero(3);
      _acceleration.reset(new AccelerationType(_frame, "ZERO"));
      _velocity.reset(new VelocityType(_frame));
    }

    Acceleration(Acceleration&& other): DynamicPoint(other), _frame(other._frame){
      _point.setZero(3);
      _acceleration.reset(new AccelerationType(_frame, "ZERO"));
      _velocity.reset(new VelocityType(_frame));
    }

    using DynamicPoint::operator=;

    virtual void compute(){
      _constant = _acceleration->getWorld();
      _point = _jacobian*_state.acceleration.get() + _constant;
    }

    virtual void computeJacobian(){
      _jacobian = _velocity->getJacobian();
    }

    virtual void update(bool jacobian) {
        if(jacobian)
          computeJacobian();
        compute();
      }

  protected:
    std::unique_ptr<AccelerationType> _acceleration;
    std::unique_ptr<VelocityType> _velocity;

    mwoibn::point_handling::FramePlus _frame;
};

} // namespace package
} // namespace library

#endif
