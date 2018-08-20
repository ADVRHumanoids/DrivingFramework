#include "mwoibn/hierarchical_control/tasks/cartesian_world_task.h"


void mwoibn::hierarchical_control::tasks::CartesianWorld::updateError(){
  _last_error.noalias() = _error;
  _error.noalias() = _reference - _ik.getFullStateWorld();

}

void mwoibn::hierarchical_control::tasks::CartesianWorld::updateJacobian(){
  _last_jacobian.noalias() = _jacobian;
  _jacobian.noalias() = -_ik.getFullJacobian();
}
