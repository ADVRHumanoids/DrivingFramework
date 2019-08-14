#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__WHEEL_CONTACT_V3_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__WHEEL_CONTACT_V3_H

#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/robot_points/contact_v2.h"

namespace mwoibn
{
namespace robot_points
{

class WheelContactV3 : public ContactV2
{

public:
WheelContactV3(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, YAML::Node config);

WheelContactV3(WheelContactV3&& other)
        : ContactV2(std::move(other)), _offset_pos(other._offset_pos),
          _ref_position(other._ref_position)
{
        compute();
}

WheelContactV3(WheelContactV3& other)
        : ContactV2(other), _offset_pos(other._offset_pos),
          _ref_position(other._ref_position)
{
        compute();
}

using Point::operator=;

virtual ~WheelContactV3() {
}

virtual void _resize();


virtual const mwoibn::VectorN& getPosition()
{
        compute();
        return _point;
}

const mwoibn::VectorN& getReactionForce(){
  return _wrench.force.getWorld();
}

virtual const mwoibn::Matrix&
getPointJacobian(mwoibn::Matrix3 rotation_matrix);

virtual const mwoibn::Matrix& getWorldJacobian();

// virtual void setPosition(mwoibn::Vector3 new_state)
// {
//         _offset_pos = new_state;
// }   // PH

// it assumes the flat ground and is spherical, I should have a torus module that I am assiging to the contact point tracking and contact point
virtual void compute();

const mwoibn::VectorN& acceleration(){

  _vec_1_ = _torus->wheelAngularVelocity().head<3>();
  _vec_2_.noalias() = _vec_1_.cross(_torus->positionOffset());
  _acceleration.noalias() = _vec_1_.cross(_vec_2_);
  return _acceleration;
}

protected:
mwoibn::Vector3 _offset_pos;
std::unique_ptr<mwoibn::robot_points::TorusModel> _torus;
mwoibn::VectorN _ref_position;
mwoibn::Vector3 _vec_1_, _vec_2_;
};

} // namespace package
} // namespace library

#endif // WHEEL_CONTACT_H
