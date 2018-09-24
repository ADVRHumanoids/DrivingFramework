#ifndef GRAVITY_COMPENSATION_SIMPLE_QR_GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_SIMPLE_QR_GRAVITY_COMPENSATION_H

#include "mwoibn/gravity_compensation/gravity_compensation.h"
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/eigen_utils/eigen_utils.h"
#include "mwoibn/basic_controllers/lower_level_controller.h"
#include "mwoibn/dynamic_models/qr_decomposition.h"

namespace mwoibn
{
namespace gravity_compensation
{

/** @brief This class provides recomuptation of a desired trajectory from
 * link_side dynamics to motor_side dynamics. It supports only a static case.
 */
class SimpleQRGravityCompensation
        : public mwoibn::basic_controllers::LowerLevelController
{

public:
/** @brief robot constructor for an online gravity compensation
 */
SimpleQRGravityCompensation(
        mwoibn::dynamic_models::QrDecomposition& dynamic_model,
        mwoibn::robot_class::INTERFACE interface =
                mwoibn::robot_class::INTERFACE::TORQUE)
        : mwoibn::basic_controllers::LowerLevelController(
                dynamic_model.getRobot(), interface),
        _dynamic_model(dynamic_model)
{
        updateModel();
        _transform = _dynamic_model.getTransformationMatrix();
        _transformation_inverser.init(_transform, 1e-10);
        compute();
}

/** @brief robot constructor for an offline gravity compensation
 */
SimpleQRGravityCompensation(
        mwoibn::dynamic_models::QrDecomposition& dynamic_model,
        mwoibn::robot_class::Robot& robot_real,
        mwoibn::robot_class::INTERFACE interface =
                mwoibn::robot_class::INTERFACE::TORQUE)
        : mwoibn::basic_controllers::LowerLevelController(robot_real, interface),
        _dynamic_model(dynamic_model)
{
        updateModel();
        _transform = _dynamic_model.getTransformationMatrix();
        _transformation_inverser.init(_transform, 1e-10);
        _command.setZero(_robot.getDofs());
        compute();
}
virtual ~SimpleQRGravityCompensation() {
}

/** @brief update Dynamic Model of the robot according to the current state of
 * a robot stored in the class **/
void updateModel() {
        _dynamic_model.update();
}

/** @brief computes control law based on the last model update
 *
 * @see updateModel()
 **/
virtual void compute()
{

        if (_dynamic_model.changed())
                _transform = _dynamic_model.getTransformationMatrix();
        else
                _transform.noalias() =
                        _dynamic_model.getTransformationMatrix();  // size may chenge here

        for (int i = 0; i < _dynamic_model.getRobot().getActuationState().size();
             i++)
                if (!_dynamic_model.getRobot().getActuationState()[i])
                        _transform.col(i).setZero();

        //std::cout << "_transform" << std::endl;
        //std::cout << _transform << std::endl;
        _transformation_inverser.compute(_transform);
        //std::cout << "_transform_invers" << std::endl;
        //std::cout << _transformation_inverser.get() << std::endl;

        _command.noalias() =
                _transformation_inverser.get() * _dynamic_model.getGravityUnconstrained();

        //std::cout << "command\t" << _command.transpose() << std::endl;
        //std::cout << "_transformation_inverser.get()\t" << _transformation_inverser.get() << std::endl;

}

/** @brief updates both model and controller and returns newly computed
 * command */
virtual const mwoibn::VectorN& update()
{
        updateModel();
        compute();
        setCommand();
        return getCommand();
}

protected:
mwoibn::dynamic_models::QrDecomposition& _dynamic_model;
mwoibn::PseudoInverseLimited _transformation_inverser;
mwoibn::MatrixLimited _transform;
};
} // namespace package
} // namespace library

#endif
