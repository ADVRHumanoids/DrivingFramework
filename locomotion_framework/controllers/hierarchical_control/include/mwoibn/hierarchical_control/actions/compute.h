#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H

#include "mwoibn/hierarchical_control/actions/basic_action.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class Compute : public Basic {
public:
Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, mwoibn::VectorN gains, double damping, mwoibn::Matrix& P, mwoibn::VectorN& command) : Basic(task), _gains(gains), _P(P), _command(command){
        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, mwoibn::Matrix& P, mwoibn::VectorN& command) : Basic(task), _P(P), _command(command){
        _gains.setConstant(_task.getTaskDofs());
        _init(damping);
}

~Compute(){
}

virtual void run(){
        _errors.noalias() = -(_gains.asDiagonal() * _task.getError());
        _errors.noalias() -= _task.getJacobian() * _command;

        _inverser_ptr->compute(_task.getJacobian(), _P);
        _command.noalias() += _inverser_ptr->getInverse() * _errors;
}

virtual actions::Compute& baseAction(){
        return *this;
}

bool updateGain(double gain){
        _gains.setConstant(gain);
        return true;
}

bool updateGain(const mwoibn::VectorN& gain){
        if(gain.size() != _gains.size()) return false;

        _gains.noalias() = gain;
        return true;
}

protected:
mwoibn::VectorN _gains, _errors, &_command;
mwoibn::Matrix& _P;
std::unique_ptr<mwoibn::Projection> _inverser_ptr;

virtual void _init(double damping){
        if (!_task.getTaskSize()) {
                throw(std::invalid_argument("Cannot initialize actions::Copmute for a zero size task."));
        }


        _errors.setZero(_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(_task.getTaskSize(), _task.getTaskDofs());
        _inverser_ptr.reset( new mwoibn::Projection(jacobian, damping));
}
};

}
} // namespace package
} // namespace library
#endif
