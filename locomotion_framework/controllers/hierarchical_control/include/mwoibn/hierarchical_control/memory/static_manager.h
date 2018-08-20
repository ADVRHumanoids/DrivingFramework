#ifndef __MWOIBN_HIERARCHICAL_CONTROL_MEMORY__STATIC_MANAGER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_MEMORY__STATIC_MANAGER_H

#include "mwoibn/hierarchical_control/memory/action_stack.h"
#include "mwoibn/hierarchical_control/memory/manager.h"

namespace mwoibn {
namespace hierarchical_control {

namespace actions {
// class Basic;
class Snap;
class DynamicReplace;
class StaticReplace;
class Secondary;
}

namespace memory {

class StaticManager : public Manager
{
public:
StaticManager() : Manager(){
}

virtual ~StaticManager() {
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

private:
ActionStack<actions::Snap> _snap;
ActionStack<actions::Secondary> _secondary;

protected:
ActionStack<actions::StaticReplace> replace;

};
}
} // namespace package
} // namespace library

#endif
