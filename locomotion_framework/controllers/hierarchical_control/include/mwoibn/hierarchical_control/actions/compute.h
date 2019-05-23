#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_COMPUTE_H

#include "mwoibn/hierarchical_control/actions/primary.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/state.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class Compute : public Primary {
public:
Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, double damping, hierarchical_control::State& state) : Primary(task, state.memory),_gains(gains), _P(state.P), _command(state.command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, const mwoibn::VectorN& damping, hierarchical_control::State& state) : Primary(task, state.memory),_gains(gains), _P(state.P), _command(state.command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, hierarchical_control::State& state) : Primary(task, state.memory), _P(state.P), _command(state.command){
        _gains.setConstant(_task.getTaskSize(), gain);
        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, hierarchical_control::State& state, memory::Manager& memory) : Primary(task, memory), _P(state.P), _command(state.command){
        _gains.setConstant(_task.getTaskSize(), gain);
        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, double damping, hierarchical_control::State& state, memory::Manager& memory) : Primary(task, memory),_gains(gains), _P(state.P), _command(state.command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

        _init(damping);
}

Compute(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gains, const mwoibn::VectorN& damping, hierarchical_control::State& state, memory::Manager& memory) : Primary(task, memory),_gains(gains), _P(state.P), _command(state.command){
        if(gains.size() != task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, sizes incompatible between gains and task."));
        if(_command.size() != task.getTaskDofs())
                throw(std::invalid_argument("hierarchical_control::controllers::Compute - couldn't add the task, incompatible sizes of task and controller."));

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
        _errors.noalias() = -(_gains.asDiagonal() * _task.getError() + _task.getVelocity());
        // std::cout << "_errors\t" << _errors.transpose() << std::endl;
        // std::cout << "velocity\t" << _task.getVelocity().transpose() << std::endl;

        _errors.noalias() -= _task.getJacobian() * _command;
        // std::cout << "_errors2\t" << _errors.transpose() << std::endl;

        _inverser_ptr->compute(_task.getJacobian(), _P);

        // std::cout << "inverse jacobian\n" << _inverser_ptr->getInverse() << std::endl;
        // std::cout << "_P\n" << _P << std::endl;
        // std::cout << "_task.getJacobian()\n" << _task.getJacobian() << std::endl;
        // std::cout << "J*P\n" << _task.getJacobian()*_P << std::endl;

        _command.noalias() += _inverser_ptr->getInverse() * _errors;
        // std::cout << "command\t" << (_task.getJacobian()*_command).transpose() << std::endl;

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
