#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_IDLE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_IDLE_H

#include "mwoibn/hierarchical_control/actions/primary.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Idle : public Primary {
public:
Idle(tasks::BasicTask& task, memory::Manager& memory) : Primary(task, memory){
}

virtual ~Idle(){
}

virtual void run(){
}




};

}
} // namespace package
} // namespace library
#endif
