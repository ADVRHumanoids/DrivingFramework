#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_WHEELS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_WHEELS_H

#include "mwoibn/hierarchical_control/controllers/continous.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn {
namespace hierarchical_control {
namespace controllers {
class Wheels : public Continous
{
public:
Wheels(mwoibn::robot_class::Robot& robot, mwoibn::hierarchical_control::tasks::BasicTask& task_high,
       mwoibn::hierarchical_control::tasks::BasicTask& task_low, double mu = 10000000)
        : Continous(robot, mu), _task_high(task_high), _task_low(task_low)
{
        _size = task_high.getTaskSize();
        _last_state.setConstant(_size, true);
        _state.setConstant(_size, true);
        _values.setZero(_size);
}

virtual void init(){
        _findTasks();
}

virtual ~Wheels() {
}
virtual void compute();

protected:
virtual bool _checkStack();
void _findTasks();
//  virtual void _updateTask(int i, mwoibn::hierarchical_control::BasicTask* task);

mwoibn::hierarchical_control::tasks::BasicTask &_task_low, &_task_high;
mwoibn::VectorBool _last_state, _state;
mwoibn::VectorN _values;
int _id_low, _id_high, _size;
};
}
} // namespace package
} // namespace library

#endif
