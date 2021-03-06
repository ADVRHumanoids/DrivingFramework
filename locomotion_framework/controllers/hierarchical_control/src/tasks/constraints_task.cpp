#include "mwoibn/hierarchical_control/tasks/constraints_task.h"

void mwoibn::hierarchical_control::tasks::Constraints::updateError() {}

void mwoibn::hierarchical_control::tasks::Constraints::updateJacobian()
{

  _last_jacobian.noalias() = _jacobian;
  _jacobian.setZero();

  int i = 0;
  for(auto&& [contact, is]:  ranges::view::zip(_robot.contacts(), _selector )){
    if(!is) continue;
    _jacobian.middleRows(i, contact->jacobianSize()) = contact->getJacobian();
    i += contact->jacobianSize();
  }



  //
  // int row = 0;
  // int size;
  // for (int i = 0; i < _selector.size(); i++)
  // {
  //   size = _robot.contacts().contact(i).jacobianSize();
  //
  //   if (!_selector[i])
  //   {
  //     _jacobian.block(row, 0, size, _robot.getDofs()).setZero();
  //   }
  //   else
  //   {
  //     for(auto& dof: dof_selector){
  //       _jacobian.block(row+dof, 0, 1, _robot.getDofs()).setZero();
  //     }
  //   }
  //
  //   row += size;
  // }
}
