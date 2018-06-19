#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_MERGE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_MERGE_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/compute.h"
#include "mwoibn/hierarchical_control/actions/replace.h"
#include "mwoibn/hierarchical_control/controllers/memory_manager.h"
#include "mwoibn/hierarchical_control/actions/snap.h"

#include <unordered_map>
#include <boost/dynamic_bitset.hpp>

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

class Merge : public Primary
{
public:
Merge(actions::Compute& main_task, actions::Compute& secondary_task, TaskMap& map, double eps, mwoibn::VectorN& command, mwoibn::VectorN& P, double dt) : Primary(main_task.getTask(), _local_memory), _command(command), _P(P), _secondary(secondary_task), _eps(eps), _primary(main_task), _snap(actions::Snap(_P, _command, _memory)), _map(map){

        if(_task.getTaskSize() != _secondary.getTask().getTaskSize())
                throw(std::invalid_argument("Couldn't intialize Merge task, incompatibile tasks sizes."));

        unsigned int size = _task.getTaskSize();

        mwoibn::VectorBool id(size);
        mwoibn::VectorN gain(size);
        mwoibn::VectorN damping(size);

        _support_tasks.assign(std::pow(2,size), tasks::BasicTask(size, _task.getTaskDofs()));

        for (int i = 0; i < std::pow(2,size ); i++) {
                boost::dynamic_bitset<> b(size, i);
                for(int j = 0; j < size; j++) {
                        id[j] = b[j];
                        gain[j] = id[j] ? _primary.gain()[j] : secondary_task.gain()[j];
                        damping[j] = id[j] ? _primary.damping(j) : secondary_task.damping(j);

                }
                _support_actions.insert(std::make_pair(id, actions::Compute(_support_tasks[i], gain, damping, _P, _command, _local_memory)));
        }

        _active_stack.reserve(size);
        _replace.assign(_support_actions.size(), actions::Replace(_P, _command, _p, dt, _memory, _map));
        _local_memory.release(_replace);

}

virtual ~Merge() {
}

virtual void run(){
        _snap.run();
        _primary.run();
        _check();
        if (_current_id.all() && _active_stack.empty()) return;

        _snap.restore();

        if(_support_actions[_current_id] != *(_active_stack.back())) _add();
}



protected:
memory::Manager _local_memory;

mwoibn::VectorN _command;
mwoibn::Matrix _P;
actions::Compute &_primary,  &_secondary;
mwoibn::VectorBool _current_id;
std::vector<tasks::BasicTask> _support_tasks;
std::unordered_map<mwoibn::VectorBool, actions::Compute, mwoibn::eigen_utils::Hasher> _support_actions;

std::vector<mwoibn::hierarchical_control::actions::Replace*> _active_stack; // how can I check active actions?
actions::Snap _snap;
std::vector<actions::Replace > _replace;
TaskMap _map;

mwoibn::Scalar _eps, _p = 5;

void _check(){
        for(int i = 0; i < _task.getTaskSize(); i++) {
                _current_id[i] = _primary.getJacobian().row(i).cwiseAbs().maxCoeff() > _eps;
        }
}

void _add(){
        _checkStack();
}

// bool _checkStack(){
//         std::find_if(_active_stack.begin(), _acti)
// }

};



}
} // namespace package
} // namespace library
#endif
