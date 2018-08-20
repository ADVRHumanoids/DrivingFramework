#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__REPLACE_MAP_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__REPLACE_MAP_H

#include "mwoibn/hierarchical_control/maps/task_map.h"
#include "mwoibn/hierarchical_control/actions/merge/local.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

namespace merge {

class ReplaceMap : public maps::TaskMap {


typedef  MapKeyIterator<tasks::BasicTask*, Local*> rep_key_iter;
typedef  MapValueIterator<tasks::BasicTask*, Local*> rep_val_iter;

ReplaceMap(){
}

virtual ~ReplaceMap(){
}


virtual actions::Task*& operator[](tasks::BasicTask& task) {
        return _map[&task];
}

virtual bool exist(tasks::BasicTask& task){
        return _map.count(&task);
}

virtual void swap(tasks::BasicTask& task, actions::Task& _new_action){
        _map[&task] = &_new_action;
}

virtual void erase(tasks::BasicTask& task){
        _map.erase(&task);
}

unsigned int size(){
        return _map.size();
}

key_iter keyBegin(){
        key_iter it = _map.begin(); return it;
}

key_iter keyEnd(){
        key_iter it = _map.end(); return it;
}

val_iter valBegin(){
        val_iter it = _map.begin(); return it;
}

val_iter valEnd(){
        val_iter it = _map.end(); return it;
}

private:
End* _current_map;
std::map<tasks::BasicTask*, Local* > _map;

};

}

}
} // namespace package
} // namespace library
#endif
