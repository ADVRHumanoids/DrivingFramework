#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_STEERING_ANGLE_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_STEERING_ANGLE_TASK_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/angles/steering.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

class SteeringAngleTask : public BasicTask
{

public:
SteeringAngleTask(std::vector<mwoibn::robot_class::angles::Steering> angels,
                  mwoibn::robot_class::Robot& robot)
        : BasicTask(), _robot(robot), _angels(angels)
{
        _init(_angels.size(), _robot.getDofs());

        _ref.setZero(_angels.size());
        _current.setZero(_angels.size());
        _resteer.setConstant(_angels.size(), false);
}

virtual ~SteeringAngleTask() {
}

virtual void updateError()
{
        _last_error.noalias() = _error;

        for (int i = 0; i < _angels.size(); i++)
        {
                _resteer[i] = false;
                _angels[i].update();

                mwoibn::eigen_utils::wrapToPi(_ref[i]);
                _error[i] = _ref[i] - _angels[i].get();

                _limit2PI(i);

                mwoibn::eigen_utils::wrapToPi(_error[i]);
                mwoibn::eigen_utils::wrapToPi(_ref[i]);

                if ( _error[i] > 0 &&  std::fabs(_error[i] - mwoibn::PI) < 50*mwoibn::PI/180) {
                        _error[i]-= mwoibn::PI;
                        _resteer[i] = true;

                }

                else if (_error[i] < 0 &&  std::fabs(_error[i] + mwoibn::PI) < 50*mwoibn::PI/180) {
                        _error[i]+= mwoibn::PI;
                        _resteer[i] = true;
                }

                if (_error[i] > 30*mwoibn::PI/180)
                        _error[i] = 30*mwoibn::PI/180;
                else if (_error[i] < -30*mwoibn::PI/180)
                        _error[i] = -30*mwoibn::PI/180;

        }
}

const mwoibn::VectorN& getCurrent()
{

        for (int i = 0; i < _angels.size(); i++)
                _current[i] = _angels[i].get();

        return _current;
}

virtual void updateJacobian()
{
        _last_jacobian.noalias() = _jacobian;

        for (int i = 0; i < _angels.size(); i++)
        {
                _jacobian.row(i) = -_angels[i].getJacobian();
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

int size() {
        return _angels.size();
}

bool resteer(int i){
        return _resteer[i];
}
const mwoibn::VectorBool& resteer(){
        return _resteer;
}
protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _ref, _current;
std::vector<mwoibn::robot_class::angles::Steering> _angels;
mwoibn::VectorBool _resteer;

void _limit2PI(int i){

        if(_error[i] - _last_error[i] > mwoibn::PI) {
                if(i == 0)
//      std::cout << i << "\t" << _error[i] << "\t" << _last_error[i] << "\t" << _ref[i];

                        _ref[i] -= mwoibn::TWO_PI;
                _error[i] = _ref[i] - _angels[i].get();
                if(i == 9)
//        std::cout << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;
                        _limit2PI(i);
        }
        else if (_last_error[i] - _error[i] > mwoibn::PI) {
                if (i == 0)
//      std::cout << i << "\t" << _error[i] << "\t" << _last_error[i] << "\t" << _ref[i];
                        _ref[i] += mwoibn::TWO_PI;
                _error[i] = _ref[i] - _angels[i].get();
                if(i == 0)
//        std::cout << "\t" << _ref[i] << "\t" << _error[i] << std::endl;
                        _limit2PI(i);
        }
}
};
}
} // namespace package
} // namespace library
#endif
