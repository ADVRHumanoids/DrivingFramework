#ifndef __MWOIBN_HIERARCHICAL_CONTROL_MAPS__ACTIONS_MAP_H
#define __MWOIBN_HIERARCHICAL_CONTROL_MAPS__ACTIONS_MAP_H

#include "mwoibn/hierarchical_control/maps/task_map.h"

namespace mwoibn {
namespace hierarchical_control {
namespace maps {

class ActionsMap : public TaskMap {

public:
ActionsMap() : TaskMap(){
}

virtual ~ActionsMap(){
}

typedef  MapKeyIterator<tasks::BasicTask*, actions::Task*> key_iter;
typedef  MapValueIterator<tasks::BasicTask*, actions::Task*> val_iter;

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

std::map<tasks::BasicTask*, actions::Task* > _map;



};
}
}
}
#endif
