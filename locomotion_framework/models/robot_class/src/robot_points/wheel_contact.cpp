#include "mwoibn/robot_points/wheel_contact.h"

mwoibn::robot_points::WheelContactV2::WheelContactV2(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, YAML::Node config)
        : ContactV2(model, state, config)
{

        if (!config["offset"])
                _readError("offset");

        std::vector<double> temp = config["offset"].as<std::vector<double> >();
        if (temp.size() == 3)
        {
                _offset_pos = mwoibn::Vector3::Map(temp.data(), temp.size());
        }
        else
        {
                std::stringstream errMsg;
                errMsg << "Expected size of field \"offset \" is 3. Got "
                       << temp.size() << " instead for contact " << config["name"]
                       << std::endl;
                throw(std::invalid_argument(errMsg.str().c_str()));
        }

        _ref_position = _frame.getLinearFixed();

        _state_size = 3;
        _resize();
        compute();
}

void mwoibn::robot_points::WheelContactV2::_resize()
{
        _jacobian.setZero(_state_size, _state.size());
        _rotation.setZero(_state_size, _state_size);
        _transformation.setZero(_state_size, _state.size());

        if (_directions.rows() == 6 && _directions.cols() == 6)
                _directions = _directions.bottomRightCorner<3, 3>();
}

const mwoibn::Matrix&
mwoibn::robot_points::WheelContactV2::getPointJacobian(mwoibn::Matrix3 rotation_matrix)
{
        compute(); //??? can I remove it savely?
        _jacobian.setZero();

        if (!_is_active)
                return _jacobian;

        _rotation.noalias() = _directions * rotation_matrix;
        _transformation.noalias() = _frame.getPositionJacobian();
        _jacobian.noalias() = _rotation * _transformation;

        return _jacobian;
}

void mwoibn::robot_points::WheelContactV2::compute()
{
        _frame.setLinearFixed(_ref_position);

        _point.noalias() = _frame.getLinearWorld();

        _point.noalias() += _offset_pos;

        _frame.setLinearWorld(_point);

        //_wrench.setLinearFixed(_frame.getLinearFixed());
}
