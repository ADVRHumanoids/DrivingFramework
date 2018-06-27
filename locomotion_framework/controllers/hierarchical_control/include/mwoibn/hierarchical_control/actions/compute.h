#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H

#include "mwoibn/hierarchical_control/actions/primary.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class Compute : public Primary {
public:
Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, double damping, mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory) : Primary(task, memory),_gains(gains), _P(P), _command(command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, const mwoibn::VectorN& damping, mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory) : Primary(task, memory),_gains(gains), _P(P), _command(command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory) : Primary(task, memory), _P(P), _command(command){
        _gains.setConstant(_task.getTaskSize(), gain);
        _init(damping);
}

Compute(const Compute& other) : Primary(other), _gains(other._gains), _P(other._P), _command(other._command), _errors(other._errors), _inverser_ptr(std::move(other._inverser_ptr.get())){
}
// Compute(Compute& other) : Primary(other), _gains(other._gains), _P(other._P), _command(other._command), _errors(other._errors), _inverser_ptr(other._inverser_ptr){
// }

Compute(Compute&& other) : Primary(std::move(other)), _gains(other._gains), _P(other._P), _command(other._command), _errors(other._errors), _inverser_ptr(std::move(other._inverser_ptr)){

}

virtual ~Compute(){
}

virtual void run(){
        _errors.noalias() = -(_gains.asDiagonal() * _task.getError());
        _errors.noalias() -= _task.getJacobian() * _command;

        _inverser_ptr->compute(_task.getJacobian(), _P);
        _command.noalias() += _inverser_ptr->getInverse() * _errors;
}

virtual void release(){
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

const mwoibn::VectorN& gain(){
        return _gains;
}

const mwoibn::Matrix& getJacobian() const {
        return _inverser_ptr->getJacobian();
}

mwoibn::Scalar damping(int i){
        return _inverser_ptr->damping(i);
}

protected:
mwoibn::VectorN _gains, _errors, &_command;
mwoibn::Matrix& _P;
std::unique_ptr<mwoibn::Projection> _inverser_ptr;

virtual void _init(const mwoibn::VectorN& damping){
        if (!_task.getTaskSize()) {
                throw(std::invalid_argument("Cannot initialize actions::Compute for a zero size task."));
        }


        _errors.setZero(_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(_task.getTaskSize(), _task.getTaskDofs());
        _inverser_ptr.reset( new mwoibn::Projection(jacobian, damping));
}

virtual void _init(double damping){
        if (!_task.getTaskSize()) {
                throw(std::invalid_argument("Cannot initialize actions::Compute for a zero size task."));
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
