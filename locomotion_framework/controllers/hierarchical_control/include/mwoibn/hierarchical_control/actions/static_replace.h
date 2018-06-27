#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__STATIC_REPLACE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS__STATIC_REPLACE_H

#include "mwoibn/hierarchical_control/actions/replace.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

/**
 * Replace an old task with a new task
 * ensures a continuity if the replacement happens at the stack end
 *
 */
class StaticReplace : public Replace {
public:
StaticReplace(actions::Task& task_new, actions::Task& task_old, mwoibn::Matrix& P, mwoibn::VectorN& command, double mu, double dt, memory::Manager& memory, maps::TaskMap& map) :
        Replace(task_new, task_old, P, command, memory, map), _dt(dt), _mu(mu){
        _setLimit();
}

StaticReplace(mwoibn::Matrix& P, mwoibn::VectorN& command, double mu, double dt, memory::Manager& memory, maps::TaskMap& map) :
        Replace(P, command, memory, map), _dt(dt), _mu(mu){
        _setLimit();
}

const StaticReplace& operator=(const StaticReplace& replace){
        return replace;
}

virtual ~StaticReplace(){
}

using Replace::start;

virtual void start(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap, double mu){
        _mu = mu;
        _setLimit();
        assign(task_new, task_old, snap);
}

virtual void setGain(double mu){
        _mu = mu; _setLimit();
}

virtual void reset(){
        _t = 0;
        _p = 0;
}

virtual void run(){

        _progress();

        Replace::run();

}
virtual bool isDone() {
        return _p < (1 - _eps);
}

virtual void release(){
        Replace::release();
        _p = 1;
}


protected:
double _p, _mu, _t, _dt, _eps = 1e-8;

virtual void _progress(){
        _t += _dt;
        // _p = 1 - std::exp(-_mu*_t);
        _p = std::tanh(_mu*_t);
}

virtual void _setLimit(){
        _t = _dt;
        _progress();
        _eps = _p; // set end to the same step as the first one
        reset();
}

virtual void _merge(){
        _command.noalias() = _q_new*_p;
        _command.noalias() += _q_old*(1-_p);
}
};

}
} // namespace package
} // namespace library
#endif
