#ifndef __MWOIBN_HIERARCHICAL_CONTROL_SHAPE_ACTIONS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_SHAPE_ACTIONS_H

#include "mwoibn/hierarchical_control/actions/primary.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/state.h"
#include "mgnss/higher_level/qp/tasks/qr_task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class ShapeAction : public Primary {
public:
ShapeAction(mgnss::higher_level::QrTask& task, mwoibn::hierarchical_control::tasks::BasicTask& reference_task, hierarchical_control::State& state) : Primary(task, state.memory), _state(state), _qr_task(task), _reference_task(reference_task){ }


virtual void run(){
    _qr_task._update();
    _qr_task.solve();
    for(auto& equality: _qr_task.equality){
      std::cout << "equality state\t" << equality->state.transpose() << std::endl;
    }
    // std::cout << "shape solution\t" << _qr_task.get().transpose() << std::endl;
    //_state.command = _qr_task.get();
    // std::cout << "final solution\t" << _state.command.transpose() << std::endl;
}

virtual void release(){ }

protected:
  hierarchical_control::State& _state;
  mgnss::higher_level::QrTask& _qr_task;
  mwoibn::hierarchical_control::tasks::BasicTask& _reference_task;
};

}
} // namespace package
} // namespace library
#endif
