#include "mwoibn/motor_side_reference/sea_reference.h"

namespace mwoibn
{

namespace motor_side_reference
{

SeaReference::SeaReference(
        mwoibn::robot_class::Robot& robot,
        mwoibn::basic_controllers::LowerLevelController& gravity_compensation,
        mwoibn::Interface interface)
        : mwoibn::basic_controllers::LowerLevelController(robot, interface),
        _gravity_compensation(gravity_compensation)
{
        _command.resize(_robot.getDofs());
        _stiffness_matrix_inverser.init(_robot.actuators().getStiffnessMatrix(), 1e-10);
        updateStiffnessMatrix();
}

void SeaReference::updateStiffnessMatrix()
{
        _stiffness_matrix_inverser.compute(_robot.actuators().getStiffnessMatrix());

        //  _selection = _robot.actuators().getActuationTypes(
        //      {mwoibn::robot_class::ACTUATOR_TYPE::ELASTIC});

        _stiffness_matrix = _stiffness_matrix_inverser.get();
}

void SeaReference::compute()
{
        // use robot state as a feedback? or command -- I don't have to pass it
        _command = _stiffness_matrix * _gravity_compensation.getCommand() + _robot.command.position.get();
}

const mwoibn::VectorN& SeaReference::update()
{

        _gravity_compensation.update();
        compute();
        setCommand();

        return getCommand();
}
} // namespace package
} // namespace library
