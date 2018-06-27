#include "mwoibn/hierarchical_control/actions/merge/end.h"
#include "mwoibn/hierarchical_control/actions/merge/replace.h"
#include "mwoibn/hierarchical_control/actions/merge.h"

mwoibn::hierarchical_control::actions::merge::End::End(mwoibn::Matrix& P, mwoibn::VectorN& command, MergeManager& memory, maps::TaskMap& map, actions::Merge& merge, double dt, double mu) : Local(memory, map, merge), _gain(), _this(P, command, _merge_memory, map, _gain), _dt(dt), _mu(mu){

        _this_memory.release(_this);
        _gain.setZero(command.size());
}

mwoibn::hierarchical_control::actions::merge::End::End(const End& other) : Local(other), _gain(other._gain), _t(other._t), _dt(other._dt), _mu(other._mu), _p(other._p), _this_memory(other._this_memory), _this(other._this, _gain) {

}

mwoibn::hierarchical_control::actions::merge::End::~End(){
}

bool mwoibn::hierarchical_control::actions::merge::End::isParent(actions::Task& task){
        return false;
}

const double mwoibn::hierarchical_control::actions::merge::End::getGain(){
        return _p;
}

void mwoibn::hierarchical_control::actions::merge::End::updateGain(){
        _p = std::tanh(_mu*_t);
        _gain.setConstant(_p);
}

void mwoibn::hierarchical_control::actions::merge::End::reset(){
        _p = 0;
        _t = 0;
        _gain.setZero();
}

void mwoibn::hierarchical_control::actions::merge::End::progress(){
        _t += _dt;
        updateGain();
}

void mwoibn::hierarchical_control::actions::merge::End::setProgress(double p){
        _t = std::atanh(p);
        updateGain();
}

void mwoibn::hierarchical_control::actions::merge::End::_setLimit(){
        _t = _dt;
        progress();
        _this.setCondition(_p);         // set end to the same step as the first one
        reset();
}

void mwoibn::hierarchical_control::actions::merge::End::run(){
        progress();
        _this.run();
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::End::swapFromFront(Local& old){
        actions::merge::Front* _base_1  = _merge_memory.local_front.get();
        actions::merge::End* _end_1 = _merge_memory.local_end.get();

        _base_1->assign(_this.action(), _end_1);
        _end_1->assign(old.action(), *_base_1);

        _map[_base_1->action().baseAction().getTask()] = _base_1;
        _map[_end_1->action().baseAction().getTask()] = _end_1;

        _end_1->setProgress(1 - _p);

        old.release();
        release();
        _merge.setLast(*_end_1);

        return *_end_1;
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::End::swapFromReplace(Local& lower){
        return lower.swapToEnd(*this);

}

void mwoibn::hierarchical_control::actions::merge::End::_end(){
        _this.secondAction().release(); // this should release pointer
        _map.erase(_this.secondAction().baseAction().getTask());

        actions::merge::Front* ptr = _merge_memory.local_front.get();
        ptr->assign(_this.action().next(), nullptr, nullptr);
        _map[_this.baseAction().getTask()] = ptr;
        _finish(*ptr);
}



void mwoibn::hierarchical_control::actions::merge::End::release(){

        if(_this.isDone()) _end();
        else
                _map[_this.baseAction().getTask()] = _next;
        _merge_memory.release(*this);
}

void mwoibn::hierarchical_control::actions::merge::End::_finish(Local& last){
        _merge.setLast(last);
}

void mwoibn::hierarchical_control::actions::merge::End::assign(actions::Task& t_new, actions::Task& t_old){
        _this_memory.replace.get();
        _this.start(t_new, t_old, *_memory.snap.get());
        _map[t_new.baseAction().getTask()] = this;
        reset();
}


void mwoibn::hierarchical_control::actions::merge::End::assign(actions::Task& t_new, actions::Task* t_old, Local* parent){
        _this_memory.replace.get();
        _this.start(t_new, *t_old, *_memory.snap.get());
        _map[baseAction().getTask()] = this;

        reset();
}

void mwoibn::hierarchical_control::actions::merge::End::push(Local& parent){
        Replace* _ptr = _merge_memory.local_replace.get();
        _ptr->assign(action(), &_this.secondAction(), &parent);
        _map[baseAction().getTask()] = _ptr;
        release();
}

mwoibn::hierarchical_control::actions::Task& mwoibn::hierarchical_control::actions::merge::End::next(){
        actions::Task& ptr = _this.next();

        if (&ptr == &_this) return *this;

        release();

        return *_map[_this.baseAction().getTask()];
}

mwoibn::hierarchical_control::actions::Primary& mwoibn::hierarchical_control::actions::merge::End::baseAction(){
        return _this.baseAction();
}

mwoibn::hierarchical_control::actions::Task& mwoibn::hierarchical_control::actions::merge::End::action(){
        return _this.action();
}
