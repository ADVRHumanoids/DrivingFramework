#ifndef HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_WHEELS_H
#define HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_WHEELS_H

#include "mwoibn/hierarchical_control/hierarchical_controller_continous.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn
{

namespace hierarchical_control
{

class HierarchicalControllerWheels : public HierarchicalControllerContinous
{
public:
  HierarchicalControllerWheels(mwoibn::robot_class::Robot& robot, mwoibn::hierarchical_control::ControllerTask& task_high,
                               mwoibn::hierarchical_control::ControllerTask& task_low, double mu = 10000000)
      : HierarchicalControllerContinous(robot, mu), _task_high(task_high), _task_low(task_low)
  {
    _size = task_high.getTaskSize();
    _last_state.setConstant(_size, true);
    _state.setConstant(_size, true);
    _values.setZero(_size);
  }

  virtual void init(){
    _findTasks();
  }

  ~HierarchicalControllerWheels() {}
  virtual void compute();

protected:
  virtual bool _checkStack();
  void _findTasks();
//  virtual void _updateTask(int i, mwoibn::hierarchical_control::ControllerTask* task);

  mwoibn::hierarchical_control::ControllerTask &_task_low, &_task_high;
  mwoibn::VectorBool _last_state, _state;
  mwoibn::VectorN _values;
  int _id_low, _id_high, _size;
};

} // namespace package
} // namespace library

#endif
