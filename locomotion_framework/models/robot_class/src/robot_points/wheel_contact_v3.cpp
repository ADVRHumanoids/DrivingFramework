#include "mwoibn/robot_points/wheel_contact_v3.h"
#include "mwoibn/point_handling/frame_plus.h"

mwoibn::robot_points::WheelContactV3::WheelContactV3(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, YAML::Node config)
        : ContactV2(model, state, config)
{
      // std::vector<std::string> names = _robot.getLinks("wheels");
        std::string name = _readEndFrame(config);

        _torus.reset(new mwoibn::robot_points::TorusModel(
                         _model, _state, mwoibn::point_handling::FramePlus(name, _model, _state),
                         mwoibn::Axis(config["reference_axis"]["x"].as<double>(),
                                      config["reference_axis"]["y"].as<double>(),
                                      config["reference_axis"]["z"].as<double>()),
                                      config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                      getGroundNormal()));

      // _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
      //           new mwoibn::robot_points::GroundWheel(torus_->axis(), torus_->groundNormal())));

      std::cout << "contacts: " << name  << "\t" << name << std::endl;

        _resize();
        compute();
}

void mwoibn::robot_points::WheelContactV3::_resize()
{
        _state_size = 3;

        _jacobian.setZero(_state_size, _state.velocity.size());
        _rotation.setZero(_state_size, _state_size);
        _transformation.setZero(_state_size, _state.velocity.size());
        _point.setZero(_state_size);

        if (_directions.rows() == 6 && _directions.cols() == 6)
                _directions = _directions.bottomRightCorner<3, 3>();

        _acceleration.setZero(_state_size);
        _velocity.setZero(_state_size);
}

const mwoibn::Matrix& mwoibn::robot_points::WheelContactV3::getWorldJacobian()
{
    _jacobian.setZero();

    if (!_is_active) return _jacobian;

    _jacobian.noalias() = _frame.getPositionJacobian();

    return _jacobian;
}

const mwoibn::Matrix&
mwoibn::robot_points::WheelContactV3::getPointJacobian(mwoibn::Matrix3 rotation_matrix)
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


// this does not support the torus model - it is a spherical model
void mwoibn::robot_points::WheelContactV3::compute()
{
        _torus->compute();
        _frame.setLinearWorld(_torus->get());
        _point = _torus->get();
}
