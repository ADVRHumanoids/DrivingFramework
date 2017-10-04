#include "mwoibn/hierarchical_control/cartesian_world_task.h"


void mwoibn::hierarchical_control::CartesianWorldTask::updateError(){
  _last_error.noalias() = _error;
  _error.noalias() = _reference - _ik.getFullStateWorld();

}

void mwoibn::hierarchical_control::CartesianWorldTask::updateJacobian(){
  _last_jacobian.noalias() = _jacobian;
  _jacobian.noalias() = -_ik.getFullJacobian();
}


