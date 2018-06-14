#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_BASIC_ACTION_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_BASIC_ACTION_H

#include "mwoibn/hierarchical_control/controllers/memory_manager.h"

namespace mwoibn {
namespace hierarchical_control {


namespace actions {

class Task;

class Basic {
public:
Basic(memory::Manager& memory) : _memory(memory){
}
~Basic(){
}

virtual void run() = 0;
virtual void release() = 0;

protected:
memory::Manager& _memory;

};


}
} // namespace package
} // namespace library
#endif
