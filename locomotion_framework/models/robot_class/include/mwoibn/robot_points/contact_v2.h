#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CONTACT_V2_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CONTACT_V2_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/robot_points/contact.h"

namespace mwoibn
{

namespace robot_points
{

/** @brief This class implements the simple contact characterized by a constant state in its end-point reference frame.
 */
class ContactV2 : public Contact
{
typedef mwoibn::Vector7 State;

public:
/**
 * @param position [in] state of contact point in its reference frame
 * @param body_id [in] RBDL id number of a reference frame
 * @param model [in] reference to the RBDL model
 * @param positions [in] reference to a model state vector
 * @param is_active [in] true/false whether the contact is avtive
 * @param directions [in] set of constraint directions
 * @param type [in] selective flag - TO BE REMOVED
 * @param name [in] name of a contact
 */
template<typename Type>
ContactV2(mwoibn::point_handling::Point::Current position, Type body_id, RigidBodyDynamics::Model& model,
          const mwoibn::robot_class::State& state, bool is_active,
          mwoibn::Matrix6 directions,
          mwoibn::point_handling::Orientation::O orientation = mwoibn::point_handling::Orientation::O(0, 0, 0, 1),
          robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN,
          std::string name = "")
        : Contact(body_id, model, state, is_active, type, name)
{
          _state_size = 6;
        _resize();
        _frame.setLinearFixed(position);
        _frame.setOrientationFixed(orientation);
        _initDirections(directions);
}

/**
 * @param point [in] constraint point
 * @param model [in] reference to the RBDL model
 * @param positions [in] reference to a model state vector
 * @param is_active [in] true/false whether the contact is avtive
 * @param directions [in] set of constraint directions
 * @param type [in] selective flag - TO BE REMOVED
 * @param name [in] name of a contact
 */
ContactV2(point_handling::Frame frame, RigidBodyDynamics::Model& model,
          const mwoibn::robot_class::State& state, bool is_active,
          mwoibn::Matrix6 directions,
          robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN,
          std::string name = "")
        : Contact(frame.getBodyId(), model, state, is_active, type, name)
{
        _state_size = 6;
        _resize();
        _frame.setLinearFixed(_frame.getLinearFixed());
        _frame.setOrientationFixed(_frame.getOrientationFixed());
        _initDirections(directions);
}

ContactV2(RigidBodyDynamics::Model& model,
          const mwoibn::robot_class::State& state,
          YAML::Node config)
        : Contact(model, state, config)
{
        _state_size = 6;
        _resize();
        _read(config);
}

ContactV2(ContactV2&& other)
        : Contact(other), _directions(other._directions)
{
        mwoibn::Matrix temp_directions = _directions;
        //_state_size = 6;
        _resize();
        _directions = temp_directions;
}

ContactV2(ContactV2& other)
        : Contact(other), _directions(other._directions)
{
        //_state_size = 6;
        mwoibn::Matrix temp_directions = _directions;
        _resize();
        _directions = temp_directions;
}

virtual ~ContactV2() { }

using Point::operator=;

virtual const mwoibn::Matrix& getPointJacobian();   // PH

virtual const mwoibn::Matrix& getWorldJacobian();   // PH


virtual const mwoibn::Matrix& getPointJacobian(mwoibn::Matrix3 rotation_matrix); // contact specific


virtual const mwoibn::VectorN& getReactionForce();
virtual const mwoibn::VectorN& getPosition();

virtual void setPosition(mwoibn::Vector3 new_state)
{
        _frame.setLinearWorld(new_state);

        //_wrench.setPointWorld(new_state); // this should be done automatically now

}   // PH

virtual const mwoibn::VectorN& acceleration(){
  return _acceleration;
}

virtual const mwoibn::VectorN& velocity(){
  return _velocity;
}

protected:
//
// template<typename Type>
// ContactV2(Type body_ref, point_handling::Position point,
//           RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions,
//           bool is_active, mwoibn::Matrix6 directions,
//           robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN)
//         : BasePH(body_ref, model, positions, {point}), _is_active(is_active),
//         _type(type), _wrench(point.getBodyId(), model, point.getLinearFixed())
// {
//         _resize();
//
//         _initDirections(directions);
//         _ground_normal << 0,0,1;
// }

// ContactV2(RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions, YAML::Node config, std::string reference)
//         : BasePH(reference, model, positions), _wrench([](YAML::Node contact) {
//             if (!contact["end_frame"])
//               _readError("end_frame");
//             return contact["end_frame"].as<std::string>();
//           }(config), model)
// {
//         _resize();
//
//         _read(config);
//         _ground_normal << 0,0,1;
// }

// ContactV2(RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions, YAML::Node config, int reference)
//         : BasePH(reference, model, positions), _wrench([](YAML::Node contact) {
//             if (!contact["end_frame"])
//               _readError("end_frame");
//             return contact["end_frame"].as<std::string>();
//           }(config), model)
// {
//         _resize();
//         _read(config);
//         _ground_normal << 0,0,1;
//
// }

virtual void _resize(){
        _jacobian.setZero(_state_size, _state.velocity.size());
        _rotation.setZero(_state_size, _state_size);
        _transformation.setZero(_state_size, _state_size);
        _directions.setZero(_state_size, _state_size);
        _point.setZero(_state_size);
        _full_state.setZero(7);
        _acceleration.setZero(_state_size);
        _velocity.setZero(_state_size);
}

virtual void _initDirections(mwoibn::Matrix6 directions);
virtual void _read(YAML::Node contact);
mwoibn::Matrix _rotation, _transformation, _directions;

private:
  mwoibn::VectorN _full_state;
//mwoibn::point_handling::Position _frame;

};
} // namespace package
} // namespace library
#endif // CONTACT_V2_H
