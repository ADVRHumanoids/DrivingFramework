#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_CONTROLLER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_CONTROLLER_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/basic.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Controller : public Basic {
public:
Controller(memory::Manager& memory) : Basic(memory){
}

Controller(Controller& other) : Basic(other){
}

Controller(const Controller& other) : Basic(other){
}

virtual ~Controller(){
}

};

}
} // namespace package
} // namespace library
#endif
