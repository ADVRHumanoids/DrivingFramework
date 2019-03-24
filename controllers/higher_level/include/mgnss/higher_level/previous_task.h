#ifndef __MGNSS_HIGHER_LEVEL_PREVIOUS_TASK_H
#define __MGNSS_HIGHER_LEVEL_PREVIOUS_TASK_H

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
// #include "mwoibn/common/logger.h"

namespace mgnss
{

namespace higher_level
{

class PreviousTask: public Constraint{

  public:
    PreviousTask(const mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& previous_state): Constraint(), _task(task), _previous_state(previous_state){
        resize(_task.getTaskSize(), _task.getTaskDofs());
    }

    virtual void update(){
      _jacobian = _task.getJacobian();
      _state.noalias() = -_jacobian*_previous_state;
    }

  protected:
    const mwoibn::hierarchical_control::tasks::BasicTask& _task;
    const mwoibn::VectorN& _previous_state;
};


}
}
#endif
