#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS__MERGE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS__MERGE_H

#include "mwoibn/hierarchical_control/maps/actions_map.h"
#include "mwoibn/hierarchical_control/maps/task_map.h"
#include "mwoibn/hierarchical_control/actions/merge/merge_manager.h"

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/compute.h"
#include "mwoibn/hierarchical_control/actions/dynamic_replace.h"
#include "mwoibn/hierarchical_control/memory/manager.h"
#include "mwoibn/hierarchical_control/actions/snap.h"
#include "mwoibn/hierarchical_control/memory/action_stack.h"
#include "mwoibn/hierarchical_control/actions/merge/replace.h"
#include "mwoibn/hierarchical_control/actions/merge/local.h"
#include "mwoibn/hierarchical_control/actions/merge/front.h"
#include "mwoibn/hierarchical_control/actions/merge/end.h"
#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/state.h"

#include <unordered_map>

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

class Merge : public Primary
{
public:
Merge(actions::Compute& main_task, actions::Compute& secondary_task, hierarchical_control::State& state, mwoibn::Scalar eps);

virtual ~Merge(){
}
//
virtual void run();
void setLast(merge::Local& last){
        _last = &last;
}

virtual actions::Primary& baseAction(){
        return *this;
}

virtual actions::Task& next(){
        _last->next();
        actions::Task* ptr = _next;
        if(ptr != this) release();
        _next = this;

        return *ptr;
}

virtual void release();

protected:
merge::MergeManager _merge_memory;
mwoibn::VectorBool _current_id, _running_id;

hierarchical_control::State& _controller_state;
maps::ActionsMap _local_map;

std::vector<merge::Replace> _replace_tasks;
std::vector<merge::End> _end_tasks;
std::vector<merge::Front> _front_tasks;
std::vector<actions::Snap> _snaps;
actions::Snap _snap;

mwoibn::Matrix __jacobian;
mwoibn::VectorN __error;

actions::Compute &_primary,  &_secondary;
merge::Local* _last;

std::vector<tasks::BasicTask> _support_tasks;
std::unordered_map<mwoibn::VectorBool, actions::Compute,
                   mwoibn::eigen_utils::Hasher> _support_actions;
mwoibn::Scalar _eps, _p = 5;


// actions::Snap _snap;
// maps::TaskMap _map;
//

void _updateTasks();
void _check();
//
// void _add();

};



}
} // namespace package
} // namespace library
#endif
