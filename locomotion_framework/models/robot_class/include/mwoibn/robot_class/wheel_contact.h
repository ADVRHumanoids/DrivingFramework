#ifndef ROBOT_CLASS_WHEEL_CONTACT_H
#define ROBOT_CLASS_WHEEL_CONTACT_H

#include "mwoibn/robot_class/contact_v2.h"

namespace mwoibn
{
namespace robot_class
{

/** @brief This class provides a contact handling for a locked wheel case, i.e.
 * when the contact point is expressed in a different frame than the end of a
 * chain
 *
 * The Jacobian matrix is computed for the whole chain and the current position
 * in the end-effector frame is updated at each iteration.
 */
class WheelContact : public ContactV2
{

public:
WheelContact(RigidBodyDynamics::Model& model,
             const mwoibn::VectorN& positions, YAML::Node config)
        : ContactV2(model, positions, config,
                    config["position_frame"].as<std::string>())
{

        mwoibn::point_handling::RawFullStatesHandler ph_chain(
                "ROOT", _model, {mwoibn::point_handling::Point(
                                         config["end_frame"].as<std::string>(), _model)});

        _local_chain = ph_chain.getChain();
        _ref_position = getPointStateFixed(0).head(3);
}

WheelContact(WheelContact&& other)
        : ContactV2(std::move(other)), _ref_position(other._ref_position),
        _local_chain(other._local_chain)
{
}

WheelContact(WheelContact& other)
        : ContactV2(other), _ref_position(other._ref_position),
        _local_chain(other._local_chain)
{
}

virtual ~WheelContact() {
}

virtual const mwoibn::Matrix& getPointJacobian(mwoibn::Matrix rotation_matrix,
                                               int i = 0)
{
        update();
        return ContactV2::getPointJacobian(rotation_matrix, i);
}

virtual mwoibn::VectorN getPosition()
{
        update();
        return ContactV2::getPosition();
}   // PH

virtual void setPosition(mwoibn::Vector3 new_state)
{
        _ref_position = new_state;
}   // PH

virtual void update() {
        setPointStateReference(0, _ref_position);
}

virtual const mwoibn::VectorInt& getChain() {
        return _local_chain;
}

protected:
mwoibn::VectorN _ref_position;

mwoibn::VectorInt _local_chain;
};

class WheelContactV2 : public ContactV2
{

public:
WheelContactV2(RigidBodyDynamics::Model& model,
               const mwoibn::VectorN& positions, YAML::Node config)
        : ContactV2(model, positions, config)
{

        if (!config["offset"])
                _readError("offset");

        std::vector<double> temp = config["offset"].as<std::vector<double> >();
        if (temp.size() == 3)
        {
                _offset_pos = mwoibn::Vector3::Map(temp.data(), temp.size());
                _offset_quat.x() = 0;
                _offset_quat.y() = 0;
                _offset_quat.z() = 0;
                _offset_quat.w() = 1;
        }
        else if (temp.size() == 7)
        {
                mwoibn::Vector7 offset = mwoibn::Vector7::Map(temp.data(), temp.size());
                mwoibn::point_handling::Point::fromFullState(offset, _offset_pos,
                                                             _offset_quat);
        }
        else
        {
                std::stringstream errMsg;
                errMsg << "Expected size of field \"offset \" is 3 or 7. Got "
                       << temp.size() << " instead for contact " << config["name"]
                       << std::endl;
                throw(std::invalid_argument(errMsg.str().c_str()));
        }

        _ref_position = getPointStateFixed(0);
        update();
        _state_size = 3;
        _resize();
}

WheelContactV2(WheelContactV2&& other)
        : ContactV2(std::move(other)), _offset_pos(other._offset_pos),
        _offset_quat(other._offset_quat), _state_pos(other._state_pos),
        _state_quat(other._state_quat), _temp_quat(other._temp_quat),
        _ref_position(other._ref_position), _state(other._state)
{
        update();
        // _resize();

}

WheelContactV2(WheelContactV2& other)
        : ContactV2(other), _offset_pos(other._offset_pos),
        _offset_quat(other._offset_quat), _state_pos(other._state_pos),
        _state_quat(other._state_quat), _temp_quat(other._temp_quat),
        _ref_position(other._ref_position), _state(other._state)
{

        update();
        //_resize();

}

virtual ~WheelContactV2() {
}

virtual void _resize()
{
        _jacobian.setZero(_state_size, _positions.size());
        _rotation.setZero(_state_size, _state_size);
        _transformation.setZero(_state_size, _positions.size());

        if (_directions.rows() == 6 && _directions.cols() == 6)
                _directions = _directions.bottomRightCorner<3, 3>();

}

virtual mwoibn::VectorN getPosition()
{
        update();
        return ContactV2::getPosition();
}   // PH

virtual const mwoibn::Matrix&
getPointJacobian(mwoibn::Matrix3 rotation_matrix, int i)
{
        update();
        _jacobian.setZero();


        if (!_is_active)
                return _jacobian;

        _rotation.noalias() = _directions * rotation_matrix;
        _transformation.noalias() = _points[i]->getPositionJacobian(_positions);
        _jacobian.noalias() = _rotation * _transformation;

        return _jacobian;
}

virtual void setPosition(mwoibn::Vector3 new_state)
{
        _offset_pos = new_state;
}   // PH

// it assumes the flat ground and is spherical, I should have a torus module that I am assiging to the contact point tracking and contact point
virtual void update()
{
        setPointStateFixed(0, _ref_position);
        _state = getPointStateWorld(0);
        mwoibn::point_handling::Point::fromFullState(_state, _state_pos,
                                                     _state_quat);
        _temp_quat = _state_quat*_offset_quat;
        _state_pos += _offset_pos;
        mwoibn::point_handling::Point::toFullState(_state, _state_pos, _temp_quat);
        setPointStateWorld(0, _state);
}

protected:
mwoibn::Vector3 _offset_pos, _state_pos;
mwoibn::Quaternion _offset_quat, _state_quat, _temp_quat;
mwoibn::Vector7 _state;

mwoibn::VectorN _ref_position;
};

} // namespace package
} // namespace library

#endif // WHEEL_CONTACT_H
