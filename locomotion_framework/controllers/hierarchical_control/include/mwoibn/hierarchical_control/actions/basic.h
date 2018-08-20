#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__BASIC_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__BASIC_H

#include "mwoibn/hierarchical_control/memory/manager.h"

namespace mwoibn {
namespace hierarchical_control {


namespace actions {

class Task;

class Basic {
public:
Basic(memory::Manager& memory) : _memory(memory){
}

const Basic& operator=(const Basic& action){
        return action;
}

Basic(const Basic& other) : _memory(other._memory){
}

virtual ~Basic(){
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
