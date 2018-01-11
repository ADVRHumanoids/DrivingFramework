#include "mwoibn/hierarchical_control/center_of_mass_task_v3.h"

void mwoibn::hierarchical_control::CenterOfMassTask3::updateError()
{
  _last_error = _error;
  _error = _reference - _robot.centerOfMass().get().head(2);
//  std::cout << _error << std::endl
  //  _error = _error.cwiseProduct(_selector); // that is not needed?
//  std::cout << "com\t" << _reference.transpose() << "\t" << _error.transpose() << std::endl;
}

void mwoibn::hierarchical_control::CenterOfMassTask3::updateJacobian()
{
  _last_jacobian = _jacobian;
  _jacobian.setZero();
  for (int i = 0; i < _map.size(); i++)
  {
    if (_map[i] != mwoibn::NON_EXISTING)
      _jacobian.col(i) = -_robot.centerOfMass().getJacobian().col(_map[i]).topRows<2>();
  }
}

void mwoibn::hierarchical_control::CenterOfMassTask3::update()
{
  _robot.centerOfMass().update();
  //  _updateSelection();
  updateError();
  updateJacobian();
}
