#ifndef __MWOIBN__POINT_HANDLING__LINEAR_ACCELERATION_H
#define __MWOIBN__POINT_HANDLING__LINEAR_ACCELERATION_H

#include "mwoibn/point_handling/point.h"
#include "mwoibn/common/state.h"
#include "mwoibn/point_handling/position.h"

namespace mwoibn
{

namespace point_handling
{

class LinearAcceleration: public Point
{

public:

  LinearAcceleration(unsigned int body_id, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        Point::Linear position = Point::Linear::Zero(3),
        std::string name = "")
      : Point(body_id, model, state, name), _frame(position, body_id, model, state)
  {
    _init();
  }

  LinearAcceleration(std::string body_name, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        Point::Linear position = Point::Linear::Zero(3),
        std::string name = "")
      : Point(body_name, model, state, name), _frame(position, body_name, model, state)
  {
    _init();
  }

  LinearAcceleration(Point::Linear linear, unsigned int body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        Point::Linear position = Point::Linear::Zero(3),
        std::string name = "")
      : Point(linear, body_id, model, state, name), _frame(position, body_id, model, state)
  {
    _init();
  }

  LinearAcceleration(Point::Linear linear, std::string body_name,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        Point::Linear position = Point::Linear::Zero(3),
        std::string name = "")
      : Point(linear, body_name, model, state, name), _frame(position, body_name, model, state)
  {
    _init();
  }

  LinearAcceleration(const LinearAcceleration&& other)
      : Point(other), _jacobian(other._jacobian), _zero(other._zero), _shift(other._shift), _frame(other._frame)  {  }

  LinearAcceleration(const LinearAcceleration& other)
      : Point(other), _jacobian(other._jacobian), _zero(other._zero), _shift(other._shift), _frame(other._frame)  {  }

  virtual ~LinearAcceleration() {}


  /** @brief get Position in a world frame */
  virtual const Point::Linear&
  getLinearWorld(bool update = false);

  virtual Point::Linear
  getLinearWorld(bool update = false) const;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setLinearWorld(const Point::Linear linear,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Linear&
  getLinearReference(unsigned int refernce_id, bool update = false);

  virtual void setLinearReference(const Point::Linear position,
                            unsigned int reference_id,
                            bool update = false);

  virtual const mwoibn::Matrix& getJacobian();

  void _init(){
    _linear << 0, 0, 0;
    _jacobian.setZero(3, _state.size());
    _zero.setZero(_state.size());
    _shift.setZero(_state.size());
    _shift[0] = 1;
  }

  void setPointWorld(const mwoibn::Vector3& position){_frame.setLinearWorld(position);}
  Point::Linear getPointWorld() const {return _frame.getLinearWorld();}
  Point::Linear getPointFixed() const {return _frame.getLinearFixed();}
  void setPointFixed(const mwoibn::Vector3& linear) {_frame.setLinearFixed(linear);}
  Position::Rotation getRotationWorld() const {return _frame.getRotationWorld();}

protected:
    mwoibn::Matrix _jacobian;
    mwoibn::VectorN _zero, _shift;
    Position _frame;

};

} // namespace package
} // namespace library

#endif
