#include "mwoibn/hierarchical_control/actions/merge.h"
#include <boost/dynamic_bitset.hpp>

//
mwoibn::hierarchical_control::actions::Merge::Merge(actions::Compute& main_task, actions::Compute& secondary_task,  hierarchical_control::State& state, mwoibn::Scalar eps, mwoibn::Scalar p, mwoibn::hierarchical_control::tasks::Constraints& constraints, mwoibn::robot_class::Robot& robot) : Primary(main_task.getTask(), _merge_memory), _secondary(secondary_task), _eps(eps), _primary(main_task), _snap(actions::Snap(state.P, state.command, _merge_memory)), _controller_state(state), _p(p){

        if(_task.getTaskSize() != _secondary.getTask().getTaskSize())
                throw(std::invalid_argument("Couldn't intialize Merge task, incompatibile tasks sizes."));

        unsigned int size = _task.getTaskSize();

        mwoibn::VectorBool id(size);
        mwoibn::VectorN gain(size);
        mwoibn::VectorN damping(size);

        _support_tasks.assign(std::pow(2,size), tasks::BasicTask(size, _task.getTaskDofs()));
        _support_actions.reserve(std::pow(2,size ));
        for (int i = 0; i < std::pow(2,size ); i++) {
                boost::dynamic_bitset<> b(size, i);
                for(int j = 0; j < size; j++) {
                        id[j] = b[j];
                        gain[j] = id[j] ? _primary.gain()[j] : secondary_task.gain()[j];
                        damping[j] = id[j] ? std::sqrt(_primary.damping(j)) : std::sqrt(secondary_task.damping(j));

                }
                _support_actions.insert(std::make_pair(id, actions::Compute(_support_tasks[i], gain, damping, _controller_state.P, _controller_state.command, _merge_memory)));
        }

        _front_tasks.assign(2, merge::Front(_merge_memory, _local_map, *this));
        _end_tasks.assign(2, merge::End(_controller_state.P, _controller_state.command, _merge_memory, _local_map, *this, _controller_state.dt, _p, constraints, robot));
        _replace_tasks.assign(std::pow(2, size+1), merge::Replace(_controller_state.P, _controller_state.command, _merge_memory, _local_map, *this, _controller_state.dt, _p));
        _snaps.assign(std::pow(2, size+3), actions::Snap(_controller_state.P, _controller_state.command, _merge_memory));

        _merge_memory.release(_front_tasks);
        _merge_memory.release(_end_tasks);
        _merge_memory.release(_replace_tasks);
        _merge_memory.release(_snaps);

        mwoibn::VectorBool ones;
        ones.setConstant(_task.getTaskSize(), true);
        merge::Front *ptr = _merge_memory.local_front.get();

        ptr->assign(_support_actions.at(ones), nullptr);
        _last = ptr;

        _local_map[_support_actions.at(ones).getTask()] = ptr;
        _running_id.setOnes(_task.getTaskSize());
        _current_id.setOnes(_task.getTaskSize());


        __error.setZero(_task.getTaskSize());
        __jacobian.setZero(_task.getTaskSize(), _controller_state.dofs);
}


void mwoibn::hierarchical_control::actions::Merge::run(){
        _snap.run();
        _primary.run();
        _check();

        if (_local_map.size() != local_size){
          std::cout << "stack " << _local_map.size() << std::endl;
          for(auto& support : _support_actions){
            if(_local_map.exist(support.second.getTask())) std::cout << support.first.transpose() << std::endl;
            //break;
          }


          counter = 0;
          local_size = _local_map.size();
        }
        counter++;

//        if(counter < 10)
//            std::cout << counter << "\t" << _secondary.getTask().getError() << std::endl;

//        std::cout << "runnung " << _running_id.transpose() << std::endl;

//        if (_current_id.all() && &_last->action() == &_primary) return;
        _snap.restore();
        if (_current_id == _running_id ) {
          _secondary.getTask().update();

                _updateTasks();
                //std::cout << "STACK" << std::endl;
                _last->run();
                //_last = _last->next();

                //std::cout << _controller_state.command.head<12>().transpose() << std::endl;
                return;
        }


        std::cout << _current_id.transpose() << "\t" <<   _running_id.transpose() << std::endl;

        _secondary.getTask().update();

        for(int i = 0; i < _current_id.size(); i++){
          if(_current_id[i] == 1 & _running_id[i] == 0) _reset(i);
        }
        _secondary.getTask().update();

        _read();

        actions::Task* action = &_support_actions.at(_current_id);
        if(_local_map.exist(action->baseAction().getTask())) {
                std::cout << "swap" << std::endl;
                _local_map[action->baseAction().getTask()]->swap(_last->baseAction());
        }
        else{
                //std::cout << "add" << std::endl;
                merge::End* _end =  _merge_memory.local_end.get();
                mwoibn::hierarchical_control::tasks::BasicTask& old = _last->baseAction().getTask();
                _last->push(*_end);
                _end->assign(*action, *_local_map[old]);
                _last = _end;
        }

        _updateTasks();
        _running_id.noalias() = _current_id;
        //std::cout << "STACK" << std::endl;
        _last->run();
        //std::cout << _controller_state.command.head<12>().transpose() << std::endl;

}

void mwoibn::hierarchical_control::actions::Merge::_updateTasks(){

        // simplest way for now
        // iterate through all tasks and check if it is in the make_pair

        for(auto& pair : _support_actions) {
                if (!_local_map.exist(pair.second.baseAction().getTask())) continue;
                for(int i = 0; i < pair.first.size(); i++) {
                        __error[i] = pair.first[i] ? _primary.getTask().getError()[i] : _secondary.getTask().getError()[i];

                        __jacobian.row(i) = pair.first[i] ? _primary.getTask().getJacobian().row(i) : _secondary.getTask().getJacobian().row(i);
                }
                pair.second.baseAction().getTask().updateError(__error);
                pair.second.baseAction().getTask().updateJacobian(__jacobian);

        }
}



void mwoibn::hierarchical_control::actions::Merge::_check(){
//        std::cout << "max coeffs" << std::endl;
        for(int i = 0; i < _task.getTaskSize(); i++) {
                _current_id[i] = _primary.getJacobian().row(i).cwiseAbs().maxCoeff() > _eps;
                //if(i == 0){
                  std::cout << _primary.getJacobian().row(i).cwiseAbs().maxCoeff() << "\t";
                  //std::cout << _secondary.getTask().getReference()[i] << std::endl;
                //}
//                std::cout << _primary.getJacobian().row(i).cwiseAbs().transpose() << std::endl;
 }
 std::cout << std::endl;
// std::cout << _current_id.transpose() << std::endl;
}

void mwoibn::hierarchical_control::actions::Merge::release(){

}
//
// void mwoibn::hierarchical_control::actions::Merge::_add(){
//         // _checkStack();
// }
