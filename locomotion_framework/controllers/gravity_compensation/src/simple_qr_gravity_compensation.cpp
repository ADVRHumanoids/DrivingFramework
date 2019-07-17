#include "mwoibn/gravity_compensation/simple_qr_gravity_compensation.h"

namespace mwoibn
{

namespace gravity_compensation
{


  SimpleQRGravityCompensation::SimpleQRGravityCompensation(
          mwoibn::dynamic_models::QrDecomposition& dynamic_model,
          mwoibn::Interface interface)
          : mwoibn::basic_controllers::LowerLevelController(
                  dynamic_model.getRobot(), interface),
         _dynamic_model(dynamic_model)
  {
          _dynamic_model.subscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR);
          updateModel();
          _transform = _dynamic_model.getTransformationMatrix();
          _transformation_inverser.init(_transform, 1e-10);

          compute();
  }

  /** @brief robot constructor for an offline gravity compensation
   */
  SimpleQRGravityCompensation::SimpleQRGravityCompensation(
          mwoibn::dynamic_models::QrDecomposition& dynamic_model,
          mwoibn::robot_class::Robot& robot_real,
          mwoibn::Interface interface)
          : mwoibn::basic_controllers::LowerLevelController(robot_real, interface),
         _dynamic_model(dynamic_model)
  {
         _dynamic_model.subscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR);
          updateModel();
          _transform = _dynamic_model.getTransformationMatrix();
          _transformation_inverser.init(_transform, 1e-10);
          _command.setZero(_robot.getDofs());

          compute();
  }




  void SimpleQRGravityCompensation::compute()
  {

          _transform = _dynamic_model.getTransformationMatrix();


          for (int i = 0; i < _dynamic_model.getRobot().getActuationState().size();
               i++)
                  if (!_dynamic_model.getRobot().getActuationState()[i])
                          _transform.col(i).setZero();

          //std::cout << _transform << std::endl;
          _transformation_inverser.compute(_transform);
          //std::cout << "_transform_invers" << std::endl;
          //std::cout << _transformation_inverser.get() << std::endl;

          _command.noalias() =
                  _transformation_inverser.get() * _dynamic_model.getNonlinearEffects();

          // std::cout << "command\t" << _command.transpose() << std::endl;
          // std::cout << "gravity\t" << _dynamic_model.getNonlinearEffects().transpose() << std::endl;
          // std::cout << "_transform\t" <<   _transformation_inverser.get().rows() << "\t" <<   _transformation_inverser.get().cols() << std::endl;

          //std::cout << "_transformation_inverser.get()\t" << _transformation_inverser.get() << std::endl;

  }

  /** @brief updates both model and controller and returns newly computed
   * command */
  const mwoibn::VectorN& SimpleQRGravityCompensation::update()
  {
          updateModel();
          compute();
          setCommand();
          return getCommand();
  }


} // namespace package
} // namespace library
