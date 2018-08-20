#include "mwoibn/hierarchical_control/actions/merge/replace.h"
#include "mwoibn/hierarchical_control/actions/merge.h"

//
mwoibn::hierarchical_control::actions::merge::Replace::Replace(mwoibn::Matrix& P, mwoibn::VectorN& command, MergeManager& memory, maps::TaskMap& map, actions::Merge& merge, double dt, double mu) : End(P, command, memory, map, merge, dt, mu){
}

mwoibn::hierarchical_control::actions::merge::Replace::~Replace(){
}

// void mwoibn::hierarchical_control::actions::merge::Replace::setParent(Local& parent){
//         _parent = &parent;
// }

void mwoibn::hierarchical_control::actions::merge::Replace::run(){
        // std::cout << "replace" << std::endl;
        progress();
        _this.run();
}

void mwoibn::hierarchical_control::actions::merge::Replace::progress(){

        _t += _dt;
        updateGain();
        // std::cout << "dt " << _dt << std::endl;
        std::cout << "rep_p " << _p << std::endl;

}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Replace::swapToEnd(Local& high){
        actions::merge::Replace* _rep_1  = _merge_memory.local_replace.get();
        actions::merge::End* _end_1 = _merge_memory.local_end.get();

        _rep_1->assign(high.action(), &_this.secondAction(), _end_1);
        _end_1->assign(action(), *_rep_1);

        double k = high.getGain() * _p;
        _end_1->setProgress(k - high.getGain() + 1);
        _rep_1->setProgress( k /_end_1->getGain() );

        _map[_end_1->action().baseAction().getTask()] = _end_1;
        _map[_rep_1->action().baseAction().getTask()] = _rep_1;

        high.release();
        release();
        _merge.setLast(*_end_1);
        return *_end_1;

}

bool mwoibn::hierarchical_control::actions::merge::Replace::isParent(actions::Task& task){
        if (&task == _parent) return true;
        return isParent(*_parent);
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Replace::swapToReplace(Local& high, Local& high_parent){
        actions::merge::Replace* _low_1  = _merge_memory.local_replace.get();
        actions::merge::Replace* _high_1 = _merge_memory.local_replace.get();

        _low_1->assign(high.action(), &_this.secondAction(), _parent);
        _high_1->assign(action(), _low_1, &high_parent);

        double k = high.getGain() * _p;
        _high_1->setProgress(k - high.getGain() + 1);
        _low_1->setProgress( k /_high_1->getGain() );

        _map[_low_1->action().baseAction().getTask()] = _low_1;
        _map[_high_1->action().baseAction().getTask()] = _high_1;

        high.release();
        release();

        return *_high_1;
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Replace::swapFromReplace(Local& lower){
        return lower.swapToReplace(*this, *_parent);
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Replace::swapFromFront(Local& lower){
        actions::merge::Front* _base_1  = _merge_memory.local_front.get();
        actions::merge::Replace* _rep_1 = _merge_memory.local_replace.get();

        _base_1->assign(_this.action(), _rep_1);
        _rep_1->assign(lower.action(), _base_1, _parent);

        _map[_base_1->action().baseAction().getTask()] = _base_1;
        _map[_rep_1->action().baseAction().getTask()] = _rep_1;

        _rep_1->setProgress(1 - _p);

        lower.release();
        release();

        return *_rep_1;

}

// theoretically should not happend
void mwoibn::hierarchical_control::actions::merge::Replace::push(Local& parent){
        _parent = &parent;
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Replace::pull(){
        return _parent->swapFromReplace(*this);
}

void mwoibn::hierarchical_control::actions::merge::Replace::assign(actions::Task& t_new, actions::Task* t_old, Local* parent){
        End::assign(t_new, t_old, nullptr);
        _parent = parent;

}
