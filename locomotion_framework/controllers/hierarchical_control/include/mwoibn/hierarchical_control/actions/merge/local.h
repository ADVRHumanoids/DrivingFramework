#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__LOCAL_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__LOCAL_H

#include "mwoibn/hierarchical_control/actions/merge/merge_manager.h"
#include "mwoibn/hierarchical_control/actions/task.h"
#include "mwoibn/hierarchical_control/maps/task_map.h"
// #include "mwoibn/hierarchical_control/actions/merge.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {
class Merge;

namespace merge {

class Local : public actions::Task  {

public:
Local(MergeManager &memory, maps::TaskMap& map, actions::Merge& merge);

virtual ~Local(){
}
const Local& operator=(const Local& task){
        return task;
}
virtual Local& pull() = 0;
virtual void release() = 0;
virtual void push(Local& parent) = 0;
virtual void assign(actions::Task& t_new, actions::Task* t_old, Local* parent) = 0;

virtual void run() = 0;

virtual Local& swapFromFront(Local& lower) = 0;
virtual Local& swapFromReplace(Local& lower) = 0;

virtual Local& swapToReplace(Local& high, Local& high_parent);
virtual Local& swapToEnd(Local& high);
const double getGain(){
        return 0;
}
virtual actions::Task& action() = 0;

virtual bool isParent(actions::Task& task) = 0;
virtual void swap(actions::Task& task);

virtual void setParent(Local& parent) = 0;


protected:
maps::TaskMap& _map;
MergeManager& _merge_memory;
actions::Merge& _merge;

};

}
}
} // namespace package
} // namespace library
#endif
