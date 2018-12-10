#include "mwoibn/motor_side_reference/sea_compensation.h"

namespace mwoibn
{

namespace motor_side_reference
{

SeaCompensation::SeaCompensation(
        mwoibn::robot_class::Robot& robot,
        mwoibn::Interface interface)
        : mwoibn::basic_controllers::LowerLevelController(robot, interface)
{
        _command.resize(_robot.getDofs());
        _stiffness_matrix_inverser.init(_robot.actuators().getStiffnessMatrix(), 1e-10);
        updateStiffnessMatrix();
}

void SeaCompensation::updateStiffnessMatrix()
{
        _stiffness_matrix_inverser.compute(_robot.actuators().getStiffnessMatrix());
        _stiffness_matrix = _stiffness_matrix_inverser.get();
}

void SeaCompensation::compute()
{
        // use robot state as a feedback? or command -- I don't have to pass it
        _command = _stiffness_matrix * _robot.state.torque.get() + _robot.command.position.get();
}

const mwoibn::VectorN& SeaCompensation::update()
{

        //_gravity_compensation.update();
        compute();
        setCommand();

        return getCommand();
}
} // namespace package
} // namespace library
