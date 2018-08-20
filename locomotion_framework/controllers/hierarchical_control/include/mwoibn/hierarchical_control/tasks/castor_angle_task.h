#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CASTOR_ANGLE_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CASTOR_ANGLE_TASK_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/angles/caster.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

class CastorAngleTask : public BasicTask
{

public:
CastorAngleTask(std::vector<robot_class::angles::Caster> angles,
                mwoibn::robot_class::Robot& robot)
        : BasicTask(), _robot(robot), _angles(angles)
{
        _init(_angles.size(), _robot.getDofs());
        _ref.setZero(_angles.size());
        _current.setZero(_angles.size());
}

virtual ~CastorAngleTask() {
}

virtual void updateError()
{
        //    std::cout << "update" << std::endl;
        _last_error.noalias() = _error;

        for (int i = 0; i < _angles.size(); i++)
        {
                _angles[i].update();
                _error[i] = _ref[i] - _angles[i].get();
                //      std::cout << "\t" << _angels[i].get()*180/3.14;// << std::endl;
                //      std::cout << "\t" << _error[i]*180/3.14;// << std::endl;
        }

        //    std::cout << "\t" << _angels[1].get()*180/3.14;// << std::endl;
        //    std::cout << "\t" << _error[1]*180/3.14;
        eigen_utils::limitToHalfPi(
                _error); // make a bigger limit to avoid chattering
                         //    std::cout << "\t" << _error[1]*180/3.14;// << std::endl;

        //    std::cout << std::fixed << "error\t" << _error.transpose() * 180 /
        //    3.14 << "\n";
        //    std::cout << std::endl;
}

virtual void updateJacobian()
{
        _last_jacobian.noalias() = _jacobian;

        for (int i = 0; i < _angles.size(); i++)
        {
                _jacobian.row(i) = -_angles[i].getJacobian();
        }
}

const mwoibn::VectorN& getCurrent()
{

        for (int i = 0; i < _angles.size(); i++)
                _current[i] = _angles[i].get();

        return _current;
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
int size() {
        return _angles.size();
}

protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _ref, _current;
std::vector<robot_class::angles::Caster> _angles;
};
}
} // namespace package
} // namespace library
#endif
