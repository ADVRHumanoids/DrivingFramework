#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_SNAP_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_SNAP_H

#include "mwoibn/hierarchical_control/actions/controller.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

/**
 * Replace an old task with a new task
 * ensures a continuity if the replacement happens at the stack end
 *
 */
class Snap : public Controller {
public:
Snap(mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory) : Controller(memory), _P(P), _command(command){
        _P_snap = _P;
        _command_snap = _command;
}


~Snap(){
}

virtual void run(){
        _P_snap.noalias() = _P;
        _command_snap.noalias() = _command;
}

virtual void restore(){
        _P.noalias() = _P_snap;
        _command.noalias() = _command_snap;
}

virtual void release(){

}

protected:
mwoibn::Matrix& _P, _P_snap;
mwoibn::VectorN& _command, _command_snap;

};

}
} // namespace package
} // namespace library
#endif
