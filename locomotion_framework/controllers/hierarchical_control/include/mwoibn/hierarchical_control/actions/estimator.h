#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_ESTIMATOR_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_ESTIMATOR_H

#include "mwoibn/hierarchical_control/actions/secondary.h"
#include "mwoibn/hierarchical_control/state.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class Estimator : public Secondary {
public:
Estimator(hierarchical_control::State& state, actions::Task& action, maps::TaskMap& map) : Secondary(action, state.memory, map), _state(state){
  _next = this;
  _estimate = _state.command;
  _command = _state.command;
}

Estimator(const Estimator& other) : Secondary(other), _state(other._state), _estimate(other._estimate), _command(other._command){
  _next = this;
}

Estimator(Estimator&& other) : Secondary(other), _state(other._state), _estimate(other._estimate), _command(other._command){
  _next = this;
}

virtual ~Estimator(){
}

// virtual void release() // default release could cause the double update on the basic task
virtual void assign(actions::Task& action){
        _action = &action;
        _next   = &action;
}

virtual void run(){
    _next = this;

    _command = _state.command;
    //std::cout << _state.P.rows() << "\t" << _state.P.cols() << std::endl;
    _estimate = _state.P*_state.command;
    // std::cout << "command\t" << _command.transpose() << std::endl;
    // std::cout << "estimate\t" << _estimate.transpose() << std::endl;
    _state.command = _estimate;
}

virtual void restore(){
    _state.command = _state.command + _command - _estimate;
}

// virtual void release(){
//     _next = _action;
//     _map.swap(_action->baseAction().getTask(), *_action); // should this be a swap or operator
//     _memory.release(*this);
// }
//
// virtual actions::Task& next() {
//         _action = &(_action->next());
//         if(_next != this) release();
//         return *_next;
// }

protected:
hierarchical_control::State& _state;
mwoibn::VectorN _command, _estimate;
};


}
} // namespace package
} // namespace library
#endif
