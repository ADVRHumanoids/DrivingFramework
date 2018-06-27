#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_MEMORY__ACTION_STACK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_MEMORY__ACTION_STACK_H

#include "mwoibn/common/types.h"

namespace mwoibn {
namespace hierarchical_control {

namespace memory {

template<typename ActionType>
class ActionStack {

public:
ActionStack(){
}
virtual ~ActionStack(){
}

ActionType* get(){
        ActionType* _ptr = _stack.back();
        _stack.pop_back();
        return _ptr;
}

bool is(){
        if(_stack.empty()) return false;
        return true;
}

void release(ActionType& action){
        _stack.push_back(&action);
}

protected:
std::vector<ActionType* > _stack;


};



}
} // namespace package
} // namespace library

#endif
