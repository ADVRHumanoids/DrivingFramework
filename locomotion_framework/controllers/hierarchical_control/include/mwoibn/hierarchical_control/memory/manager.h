#ifndef __MWOIBN_HIERARCHICAL_CONTROL_MEMORY__MANAGER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_MEMORY__MANAGER_H

#include "mwoibn/hierarchical_control/memory/action_stack.h"
namespace mwoibn {
namespace hierarchical_control {

namespace actions {
// class Basic;
class Snap;
class Replace;
class Secondary;
class Front;
}

namespace memory {

class Manager
{
public:
Manager(){
}

virtual ~Manager() {
}

template<typename Action>
void release(std::vector<Action>& actions){
        for(auto& action : actions) release(action);
}

void release(actions::Secondary& action){
        secondary.release(action);
}
void release(actions::Snap& action){
        snap.release(action);
}
void release(actions::Replace& action){
        replace.release(action);
}

ActionStack<actions::Snap> snap;
ActionStack<actions::Replace> replace;
ActionStack<actions::Secondary> secondary;


};
}
} // namespace package
} // namespace library

#endif
