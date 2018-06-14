#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_MEMORY_MANAGER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_MEMORY_MANAGER_H

// temporary libraries
// #include <iostream>
// #include <fstream>
// #include <ctime>
// #include <chrono>
/**
 * \todo change the package name to hierarchical_control
 * \todo add to the naming convention do not name the package and internal class
 *******************************the same
 *
 */
namespace mwoibn {
namespace hierarchical_control {

namespace actions {
class Basic;
class Snap;
class Replace;
class Secondary;

}

namespace memory {

//! Implementation of a hierarchical controller
/**
 * \todo add a reference to the paper about the implemented controller
 * \todo add task names for convenience
 * \todo add some utility functions, so that the user can get some information
 *******************************about tasks (combined with the name utility)
 * \todo add basic cartesian position task
 * \todo add a constructor taking the tasks (with variable number of arguments)
 * \todo **extract the continous adjustement to the task change as an inheriting
 *******************************class**
 *
 */
class Manager
{
public:
Manager(){
}

virtual ~Manager() {
}

// template<typename Action>
// void release(Action* action){
//
// }

template<typename Action>
void release(std::vector<Action>& actions){
        for(auto& action : actions) release(action);
}
//
// void release(actions::Snap* action){
//         _idle_snap.push_back(action);
// }
// void release(actions::Replace* action){
//         _idle_replace.push_back(action);
// }
//
// void release(actions::Secondary* action){
//         _idle_secondary.push_back(action);
// }

void release(actions::Snap& action){
        _idle_snap.push_back(&action);
}
void release(actions::Replace& action){
        _idle_replace.push_back(&action);
}
void release(actions::Secondary& action){
        _idle_secondary.push_back(&action);
}

bool isSnap(){
        if(_idle_snap.empty()) return false;
        return true;
}
bool isReplace(){
        if(_idle_replace.empty()) return false;
        return true;
}
bool isSecondary(){
        if(_idle_secondary.empty()) return false;
        return true;
}

actions::Snap* getSnap(){
        actions::Snap* snap_ptr = _idle_snap.back();
        _idle_snap.pop_back();
        return snap_ptr;
}

actions::Replace* getReplace(){
        actions::Replace* rep_ptr = _idle_replace.back();
        _idle_replace.pop_back();
        return rep_ptr;
}

actions::Secondary* getSecondary(){
        actions::Secondary* sec_ptr = _idle_secondary.back();
        _idle_secondary.pop_back();
        return sec_ptr;
}

protected:
actions::Snap* _assignSnap(actions::Basic& action);

std::vector<actions::Snap* > _idle_snap;
std::vector<actions::Replace* > _idle_replace;
std::vector<actions::Secondary* > _idle_secondary;


};
}
} // namespace package
} // namespace library

#endif
