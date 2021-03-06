#include "mwoibn/hierarchical_control/tasks/center_of_mass_task_v2.h"

void mwoibn::hierarchical_control::tasks::CenterOfMass2::updateError()
{
  _last_error = _error;
  _error = _reference - _robot.centerOfMass().get().head(2);
  std::cout << "error\t" << _error.transpose() << std::endl;
}

void mwoibn::hierarchical_control::tasks::CenterOfMass2::updateJacobian()
{
  _last_jacobian = _jacobian;
  _jacobian.setZero();
  for (int i = 0; i < _map.size(); i++)
  {
    if (_map[i] != mwoibn::NON_EXISTING)
      _jacobian.col(i) = -_robot.centerOfMass().getJacobian().col(_map[i]).topRows<2>();
  }
}

void mwoibn::hierarchical_control::tasks::CenterOfMass2::update()
{
  //  _robot.centerOfMass().update();
  updateError();
  updateJacobian();
}
