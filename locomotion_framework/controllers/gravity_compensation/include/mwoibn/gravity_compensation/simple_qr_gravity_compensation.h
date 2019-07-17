#ifndef __MWOIBN__GRAVITY_COMPENSATION__SIMPLE_QR_GRAVITY_COMPENSATION_H
#define __MWOIBN__GRAVITY_COMPENSATION__SIMPLE_QR_GRAVITY_COMPENSATION_H

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
        mwoibn::Interface interface = "TORQUE");

/** @brief robot constructor for an offline gravity compensation
 */
SimpleQRGravityCompensation(
        mwoibn::dynamic_models::QrDecomposition& dynamic_model,
        mwoibn::robot_class::Robot& robot_real,
        mwoibn::Interface interface = "TORQUE");

virtual ~SimpleQRGravityCompensation() {
  _dynamic_model.unsubscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR);

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

virtual void compute();

/** @brief updates both model and controller and returns newly computed
 * command */
virtual const mwoibn::VectorN& update();

protected:
mwoibn::dynamic_models::QrDecomposition& _dynamic_model;
mwoibn::PseudoInverseLimited _transformation_inverser;
mwoibn::MatrixLimited _transform;
};
} // namespace package
} // namespace library

#endif
