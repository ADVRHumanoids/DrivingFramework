#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__DYNAMIC_REPLACE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__DYNAMIC_REPLACE_H

#include "mwoibn/hierarchical_control/actions/replace.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

/**
 * Replace an old task with a new task
 * ensures a continuity if the replacement happens at the stack end
 *
 */
class DynamicReplace : public Replace {
public:
DynamicReplace(actions::Task& task_new, actions::Task& task_old, mwoibn::Matrix& P, mwoibn::VectorN& command, double mu, memory::Manager& memory, maps::TaskMap& map, mwoibn::VectorN& gains, double cut_off = 1e-3) : Replace(task_new, task_old, P, command, memory, map), _gains(gains), _cut_off(cut_off){

        //std::cout << "command " << _command.size() << std::endl;

        _ones.setOnes(_command.size());

}

DynamicReplace(mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory, maps::TaskMap& map, mwoibn::VectorN& gains, double cut_off = 1e-3) :
        Replace(P, command, memory, map), _gains(gains), _cut_off(cut_off){

        //std::cout << "command " << _command.size() << std::endl;
        _ones.setOnes(_command.size());

}

DynamicReplace(const DynamicReplace& other) : Replace(other), _gains(other._gains), _cut_off(other._cut_off), _ones(other._ones) {

}

DynamicReplace(const DynamicReplace& other, mwoibn::VectorN& gains) : Replace(other), _gains(gains), _cut_off(other._cut_off), _ones(other._ones) {

}

const DynamicReplace& operator=(const DynamicReplace& replace){
        return replace;
}

using Replace::start;

virtual void setCondition(double cut_off){
        _cut_off= cut_off;
}

virtual void start(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap, double mu = 0){
        if(mu > 0)
                _cut_off = mu;
        // _setLimit();
        assign(task_new, task_old, snap);
}


virtual ~DynamicReplace(){
}

// finish when all elements are done
virtual bool isDone(){
        return _gains.cwiseAbs().maxCoeff() < _cut_off;
}

protected:
mwoibn::VectorN& _gains, _ones;
double _cut_off; // send it on start

virtual void _merge(){
        _command.noalias() = _q_new.cwiseProduct(_gains);
        _command.noalias() += _q_old.cwiseProduct(_ones-_gains);
}

};

}
} // namespace package
} // namespace library
#endif
