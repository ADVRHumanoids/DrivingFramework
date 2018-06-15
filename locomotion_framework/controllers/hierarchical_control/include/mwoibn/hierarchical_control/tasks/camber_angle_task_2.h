#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CAMBER_ANGLE_TASK_2_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CAMBER_ANGLE_TASK_2_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/angles/camber.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

class CamberAngleTask : public BasicTask
{

public:
CamberAngleTask(std::vector<robot_class::angles::Camber> angels, mwoibn::robot_class::Robot& robot)
        : BasicTask(), _robot(robot), _angles(angels)
{
        _init(_angles.size(), _robot.getDofs());
        _ref.setZero(_angles.size());
        _current.setZero(_angles.size());
}

virtual ~CamberAngleTask() {
}

virtual void updateError()
{
        _last_error.noalias() = _error;

        for(int i = 0; i < _angles.size(); i++) {
                _angles[i].update();
                _error[i] = _ref[i] - _angles[i].get();
        }

        eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering

}

const mwoibn::VectorN& getCurrent(){

        for(int i = 0; i < _angles.size(); i++)
                _current[i] = _angles[i].get();

        return _current;
}

virtual void updateJacobian() {
        _last_jacobian.noalias() = _jacobian;

        for(int i = 0; i < _angles.size(); i++) {
                _jacobian.row(i) = -_angles[i].getJacobian();
        }
}

virtual const mwoibn::VectorN& getReference() const {
        return _ref;
}

virtual void setReference(const mwoibn::VectorN& reference)
{
        _ref = reference;
}

virtual double getReference(int i) const {
        return _ref[i];
}
virtual void setReference(int i, double reference) {
        _ref[i] = reference;
}

protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _ref, _current;
std::vector<robot_class::angles::Camber> _angles;
};

}
} // namespace package
} // namespace library

#endif
