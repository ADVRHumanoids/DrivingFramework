#ifndef __MGNSS_HIGHER_LEVEL_TASKS_MANAGER_H
#define __MGNSS_HIGHER_LEVEL_TASKS_MANAGER_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/merge.h"
#include "mwoibn/hierarchical_control/actions/replace.h"

namespace mgnss
{

namespace higher_level
{

class MergeManager
{

public:
MergeManager(mwoibn::hierarchical_control::controllers::Actions& controller) : _controller(controller){

}
virtual ~MergeManager(){
}

void addTask(mwoibn::hierarchical_control::tasks::Merge task, double eps = 1e-8);

void initialize();

void update();

// const BasicTask& getTask(){
//         return _support_tasks[_current_id];
// }

protected:
mwoibn::VectorBool _current_id;
std::vector<std::pair<mwoibn::hierarchical_control::tasks::Merge, double > > _tasks;
std::unordered_map<mwoibn::VectorBool, BasicTask, mwoibn::eigen_utils::Hasher> _support_tasks;
mwoibn::hierarchical_control::controllers::Actions& _controller;
mwoibn::hierarchical_control::actions::Replace* _active_action;

}

};
}
}
#endif // PROGRAM_STEERING_H
