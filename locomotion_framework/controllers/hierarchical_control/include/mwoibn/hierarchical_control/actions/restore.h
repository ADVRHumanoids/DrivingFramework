#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_RESTORE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_RESTORE_H

#include "mwoibn/hierarchical_control/actions/secondary.h"
#include "mwoibn/hierarchical_control/state.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

template<typename Type>
class Restore : public Secondary {
public:
Restore(Type& action, hierarchical_control::State& state, maps::TaskMap& map) : Secondary(action, state.memory, map), _restore(action){
  _next = this;
}

Restore(const Restore& other) : Secondary(other), _restore(other._restore){
  _next = this;
 }

Restore(Restore&& other) : Secondary(other), _restore(other._restore){
  _next = this;
}

virtual ~Restore(){
}

// virtual void release() // default release could cause the double update on the basic task

virtual void run(){
  _next = this;

_restore.restore();
}

protected:
  Type& _restore;

};


}
} // namespace package
} // namespace library
#endif
