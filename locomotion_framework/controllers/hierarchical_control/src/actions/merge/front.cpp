#include "mwoibn/hierarchical_control/actions/merge/front.h"
mwoibn::hierarchical_control::actions::merge::Front::Front(MergeManager& memory, maps::TaskMap& map, actions::Merge& merge) : Local(memory, map, merge){
}

mwoibn::hierarchical_control::actions::merge::Front::~Front(){
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Front::pull(){
        return _parent->swapFromFront(*this);

}

void mwoibn::hierarchical_control::actions::merge::Front::release(){
        if(_next != this)
                _map[baseAction().getTask()] = _next;
        else
                _map.erase(baseAction().getTask());
        _merge_memory.release(*this);
}

void mwoibn::hierarchical_control::actions::merge::Front::push(Local& parent){
        _parent = &parent;
}


void mwoibn::hierarchical_control::actions::merge::Front::assign(actions::Task& t_new, Local* parent){
        _parent = parent;
        _this = &t_new;
        _next = this;
}

void mwoibn::hierarchical_control::actions::merge::Front::assign(actions::Task& t_new, actions::Task* t_old, Local* parent){
        _parent = parent;
        _this = &t_new;
        _next = this;
        _map[baseAction().getTask()] = this;
}

bool mwoibn::hierarchical_control::actions::merge::Front::isParent(actions::Task& task){
        if (&task == _parent) return true;
        return isParent(*_parent);
}

void mwoibn::hierarchical_control::actions::merge::Front::setParent(Local& parent){
        _parent = &parent;
}

void mwoibn::hierarchical_control::actions::merge::Front::run(){
        // std::cout << "front" << std::endl;
        _this->run();
}

mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Front::swapFromFront(Local& lower){
// this should not perform any swaps
        return lower;
}


mwoibn::hierarchical_control::actions::merge::Local& mwoibn::hierarchical_control::actions::merge::Front::swapFromReplace(Local& lower){
// this should not perform any swaps
        return lower;
}

mwoibn::hierarchical_control::actions::Primary& mwoibn::hierarchical_control::actions::merge::Front::baseAction(){
        return _this->baseAction();
}

mwoibn::hierarchical_control::actions::Task& mwoibn::hierarchical_control::actions::merge::Front::action(){
        return *_this;
}
