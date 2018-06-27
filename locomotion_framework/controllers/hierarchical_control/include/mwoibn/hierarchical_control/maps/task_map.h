#ifndef __MWOIBN_HIERARCHICAL_CONTROL_MAPS__TASK_MAP_H
#define __MWOIBN_HIERARCHICAL_CONTROL_MAPS__TASK_MAP_H

#include "mwoibn/hierarchical_control/actions/task.h"
#include "mwoibn/std_utils/map.h"
#include "mwoibn/common/types.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace maps {

class TaskMap {
public:
TaskMap(){
}

virtual ~TaskMap(){
}

virtual actions::Task*& operator[](tasks::BasicTask& task) = 0;

virtual bool exist(tasks::BasicTask& task) = 0;

virtual void swap(tasks::BasicTask& task, actions::Task& _new_action) = 0;

virtual void erase(tasks::BasicTask& task) = 0;

virtual unsigned int size() = 0;



};
}
}
}
#endif
