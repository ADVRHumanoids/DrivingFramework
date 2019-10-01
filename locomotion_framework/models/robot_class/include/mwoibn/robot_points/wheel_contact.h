#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__WHEEL_CONTACT_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__WHEEL_CONTACT_H

#include "mwoibn/robot_points/contact_v2.h"

namespace mwoibn
{
namespace robot_points
{

/** @brief This class provides a contact handling for a locked wheel case, i.e.
 * when the contact point is expressed in a different frame than the end of a
 * chain
 *
 * The Jacobian matrix is computed for the whole chain and the current position
 * in the end-effector frame is updated at each iteration.
 */
// class WheelLocked : public ContactV2
// {
//
// public:
// WheelLocked(RigidBodyDynamics::Model& model,
//              const mwoibn::VectorN& positions, YAML::Node config)
//         : ContactV2(model, positions, config,
//                     config["position_frame"].as<std::string>())
// {
//
//         mwoibn::point_handling::RawFullStatesHandler ph_chain(
//                 "ROOT", _model, {mwoibn::point_handling::Position(
//                                          config["end_frame"].as<std::string>(), _model)});
//
//         _local_chain = ph_chain.getChain();
//         _ref_position = getPointStateFixed(0).head(3);
// }
//
// WheelLocked(WheelLocked&& other)
//         : ContactV2(std::move(other)), _ref_position(other._ref_position),
//         _local_chain(other._local_chain)
// {
// }
//
// WheelLocked(WheelLocked& other)
//         : ContactV2(other), _ref_position(other._ref_position),
//         _local_chain(other._local_chain)
// {
// }
//
// virtual ~WheelLocked() {
// }
//
// virtual const mwoibn::Matrix& getPointJacobian(mwoibn::Matrix rotation_matrix)
// {
//         update();
//
//         return ContactV2::getPointJacobian(rotation_matrix);
// }
//
// virtual mwoibn::VectorN getPosition()
// {
//         update();
//         return ContactV2::getPosition();
// }   // PH
//
// virtual void setPosition(mwoibn::Vector3 new_state)
// {
//         _ref_position = new_state;
// }   // PH
//
// virtual void update() {
//         setPointStateReference(0, _ref_position);
//         //_wrench.setPointWorld(_ref_position);
//
// }
//
// virtual const mwoibn::VectorInt& getChain() {
//         return _local_chain;
// }
//
// protected:
// mwoibn::VectorN _ref_position;
//
// mwoibn::VectorInt _local_chain;
// };

class WheelContactV2 : public ContactV2
{

public:
WheelContactV2(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, YAML::Node config);

WheelContactV2(WheelContactV2&& other)
        : ContactV2(std::move(other)), _offset_pos(other._offset_pos),
          _ref_position(other._ref_position)
{
        compute();
}

WheelContactV2(WheelContactV2& other)
        : ContactV2(other), _offset_pos(other._offset_pos),
          _ref_position(other._ref_position)
{
        compute();
}

using Point::operator=;

virtual ~WheelContactV2() {
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

virtual void setPosition(mwoibn::Vector3 new_state)
{
        _offset_pos = new_state;
}   // PH

// it assumes the flat ground and is spherical, I should have a torus module that I am assiging to the contact point tracking and contact point
virtual void compute();

protected:
mwoibn::Vector3 _offset_pos;

mwoibn::VectorN _ref_position;
};

} // namespace package
} // namespace library

#endif // WHEEL_CONTACT_H
