#ifndef __MGNSS_CONTROLLERS_CENTRALIZED_CONTROLLER_H
#define __MGNSS_CONTROLLERS_CENTRALIZED_CONTROLLER_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/gravity_compensation/simple_qr_gravity_compensation.h>
#include <mwoibn/motor_side_reference/sea_reference.h>
#include <mwoibn/dynamic_models/qr_decomposition.h>


namespace mgnss {
namespace controllers {

class OnlineCentralizedController {

public:
  OnlineCentralizedController(mwoibn::robot_class::Robot& robot);
  virtual ~OnlineCentralizedController(){}

  void update();
  void fullUpdate(const mwoibn::VectorN& command);

protected:

  mwoibn::robot_class::Robot& _robot;

  std::unique_ptr<mwoibn::dynamic_models::QrDecomposition> _dynamic_model_ptr; // online set up
  std::unique_ptr<mwoibn::gravity_compensation::SimpleQRGravityCompensation> _gravity_compensation_ptr;
  std::unique_ptr<mwoibn::motor_side_reference::SeaReference> _actuation_model_ptr;

  bool _motor_side = false;
//  bool _valid = false;

};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
