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


virtual mwoibn::VectorN getPosition()
{
        compute();
        return _point;
}

const mwoibn::VectorN& getReactionForce(){
  return _wrench.force.getWorld();
}

virtual const mwoibn::Matrix&
getPointJacobian(mwoibn::Matrix3 rotation_matrix);

// virtual void setPosition(mwoibn::Vector3 new_state)
// {
//         _offset_pos = new_state;
// }   // PH

// it assumes the flat ground and is spherical, I should have a torus module that I am assiging to the contact point tracking and contact point
virtual void compute();

protected:
mwoibn::Vector3 _offset_pos;
std::unique_ptr<mwoibn::robot_points::TorusModel> _torus;
mwoibn::VectorN _ref_position;
};

} // namespace package
} // namespace library

#endif // WHEEL_CONTACT_H
