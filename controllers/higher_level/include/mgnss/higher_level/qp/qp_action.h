#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_QP_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_QP_H

#include "mwoibn/hierarchical_control/actions/primary.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/state.h"
#include "mgnss/higher_level/qp/tasks/qr_task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class QP : public Primary {
public:
QP(mgnss::higher_level::QrTask& task, hierarchical_control::State& state) : Primary(task, state.memory), _state(state), _qr_task(task){ }


virtual void run(){
    _qr_task._update();
    _qr_task.solve();
    std::cout << "previous solution\t" << _state.command.transpose() << std::endl;
    _state.command = _qr_task.get();
    std::cout << "final solution\t" << _state.command.transpose() << std::endl;
}

virtual void release(){ }

protected:
  hierarchical_control::State& _state;
  mgnss::higher_level::QrTask& _qr_task;
};

}
} // namespace package
} // namespace library
#endif
