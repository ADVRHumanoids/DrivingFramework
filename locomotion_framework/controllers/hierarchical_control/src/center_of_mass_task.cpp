#include "mwoibn/hierarchical_control/center_of_mass_task.h"

void mwoibn::hierarchical_control::CenterOfMassTask::updateError()
{
  _last_error = _error;
  _error = _reference - _robot.centerOfMass().get().head(2);
//  std::cout << _error << std::endl
  //  _error = _error.cwiseProduct(_selector); // that is not needed?
//  std::cout << "com\t" << _reference.transpose() << "\t" << _error.transpose() << std::endl;
}

void mwoibn::hierarchical_control::CenterOfMassTask::updateJacobian()
{
  _last_jacobian = _jacobian;
  _jacobian = -_robot.centerOfMass().getJacobian().topRows(2);

//  mwoibn::VectorBool selector = _robot.contacts().getActiveDofs();
//  for (int i = 0; i < selector.size(); i++)
//    selector[i] = selector[i] && _selector_dof[i];

//  for (int i = 0; i < _jacobian.cols(); i++)
//  {
//    _jacobian.col(i) = _jacobian.col(i) * selector.at(i);
//  }
}

void mwoibn::hierarchical_control::CenterOfMassTask::update()
{
  _robot.centerOfMass().update();
  //  _updateSelection();
  updateError();
  updateJacobian();
}
