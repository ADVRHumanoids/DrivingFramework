#include "mwoibn/hierarchical_control/tasks/orientation_world_task.h"

void mwoibn::hierarchical_control::tasks::OrientationWorld::updateError()
{
  _last_error.noalias() = _error;

//  std::vector<mwoibn::Quaternion> current = _ik.getFullStatesWorld();

  int k = 0;
  for (int i = 0; i < _ik.size(); i++)
  {
    _current = _ik.getPointStateWorld(i);

    _current.ensureHemisphere(_reference[i]);

    _skew <<                 0, -_reference[i].z(),  _reference[i].y(),
            _reference[i].z(),                  0, -_reference[i].x(),
           -_reference[i].y(),  _reference[i].x(),                  0;

    _axis = _current.axis();
    _error.segment(k, _ik.getPointJacobianRows(i)) = _reference[i].w() * _axis;
    _axis = _reference[i].axis();
    _error.segment(k, _ik.getPointJacobianRows(i)) -= _current.w() * _axis;
    _error.segment(k, _ik.getPointJacobianRows(i)) += _skew * _current.axis();

    k += _ik.getPointJacobianRows(i);

//    _previous_state[i] = _current;

  }

}

void mwoibn::hierarchical_control::tasks::OrientationWorld::updateJacobian()
{
  _last_jacobian.noalias() = _jacobian;
  _jacobian.noalias() = _ik.getFullJacobian();
}
