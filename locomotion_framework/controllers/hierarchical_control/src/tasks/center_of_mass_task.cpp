#include "mwoibn/hierarchical_control/tasks/center_of_mass_task.h"

void mwoibn::hierarchical_control::tasks::CenterOfMass::updateError()
{
        _last_error = _error;
        _error = _reference - _robot.centerOfMass().get().head(2);

//  std::cout << "com\t" <<  _robot.centerOfMass().get().transpose() << "\t" << _reference.transpose() << std::endl;

}

void mwoibn::hierarchical_control::tasks::CenterOfMass::updateJacobian()
{
        _last_jacobian = _jacobian;
        _jacobian = -_robot.centerOfMass().getJacobian().topRows(2);

        for (int i = 0; i < _selector_dof.size(); i++)
        {
                if (!_selector_dof[i])
                        _jacobian.col(i).setZero();
        }

}

void mwoibn::hierarchical_control::tasks::CenterOfMass::update()
{
        _robot.centerOfMass().update(true);
        //  _updateSelection();
        updateError();
        updateJacobian();
}
