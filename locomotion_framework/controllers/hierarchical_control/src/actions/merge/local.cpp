#include "mwoibn/hierarchical_control/actions/merge/front.h"

mwoibn::hierarchical_control::actions::merge::Local::Local(MergeManager &memory, maps::TaskMap& map, actions::Merge& merge) : Task(memory), _map(map), _merge_memory(memory), _merge(merge){
        _next = this;
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Local::swapToReplace(Local& high, Local& high_parent){
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Local::swapToEnd(Local& high){
}

void mwoibn::hierarchical_control::actions::merge::Local::swap(actions::Task& task){
        if(&task == this) return;
        if (!isParent(task)) return;

        pull().swap(task);
}
