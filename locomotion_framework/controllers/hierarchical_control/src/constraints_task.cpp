#include "mwoibn/hierarchical_control/constraints_task.h"

void mwoibn::hierarchical_control::ConstraintsTask::updateError() {}

void mwoibn::hierarchical_control::ConstraintsTask::updateJacobian()
{
//  std::cout << "update" << std::endl;
  _last_jacobian.noalias() = _jacobian;
  _jacobian.noalias() = -_robot.contacts().getJacobian();

  int row = 0;
  int size;
  for (int i = 0; i < _selector.size(); i++)
  {
    size = _robot.contacts().contact(i).jacobianSize();
//    if (_robot.contacts().contact(i).isActive())
//    if (!_selector.at(i) && _robot.contacts().contact(i).isActive())

    if (!_selector[i])
    {
      _jacobian.block(row, 0, size, _robot.getDofs()).setZero();
    }
    else
    {
      for(auto& dof: dof_selector){
        _jacobian.block(row+dof, 0, 1, _robot.getDofs()).setZero();
      }
    }

    row += size;


  }

//  std::cout << "constraint jacobian" << std::endl;
//  std::cout << _jacobian << std::endl;

}
