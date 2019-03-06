#include "mwoibn/hierarchical_control/aggravated_task.h"

  //   add task at the end of the stack
void mwoibn::hierarchical_control::AggravatedTask::addTask(ControllerTask& task){
        mwoibn::VectorBool selector;
        selector.setConstant(task.getTaskSize(), true);

        _verify(task, selector);

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        _tasks_ptrs.push_back(std::make_pair(selector, std::ref(task)));
}

void mwoibn::hierarchical_control::AggravatedTask::addTask(ControllerTask& task, mwoibn::VectorBool selector){

        _verify(task, selector);

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        _tasks_ptrs.push_back(std::make_pair(selector, std::ref(task)));
}
// add task at the end of the stack
void mwoibn::hierarchical_control::AggravatedTask::addTask(ControllerTask& task, unsigned int i){
        mwoibn::VectorBool selector;
        selector.setConstant(task.getTaskSize(), true);
        _verify(task, selector);

        if(i > _tasks_ptrs.size())
                throw(std::invalid_argument("Couldn't add task to the aggravated task, requested number is too high."));

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        _tasks_ptrs.insert(_tasks_ptrs.begin()+i, std::make_pair(selector, std::ref(task)));

}

//  // i - zero based
void mwoibn::hierarchical_control::AggravatedTask::addTask(ControllerTask& task, mwoibn::VectorBool selector, unsigned int i){
        _verify(task, selector);

        if(i > _tasks_ptrs.size())
                throw(std::invalid_argument("Couldn't add task to the aggravated task, requested number is too high."));

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        _tasks_ptrs.insert(_tasks_ptrs.begin()+i, std::make_pair(selector, std::ref(task)));
}

void mwoibn::hierarchical_control::AggravatedTask::updateJacobian(){
        _last_jacobian.noalias() = _jacobian;
        int row = 0;
        for(auto& task : _tasks_ptrs) {
                task.second.updateJacobian();
                for(int i = 0; i < task.first.size(); i++) {
                        if(!task.first[i]) continue;
                        _jacobian.row(row) = task.second.getJacobian().row(i);
                        row++;
                }
        }
}
//! generic function to provide the same syntax for error update of all
//derived classes
void mwoibn::hierarchical_control::AggravatedTask::updateError(){

        _last_error.noalias() = _error;
        int row = 0;
        for(auto& task : _tasks_ptrs) {
                task.second.updateError();
                for(int i = 0; i < task.first.size(); i++) {
                        if(!task.first[i]) continue;
                        _error[row] = task.second.getError()[i];
                        _velocity[row] = task.second.getVelocity()[i];
                        row++;
                }
        }
}
//! updates whole task in one call, calls updateError() and updateJacobin() in
//that order

void mwoibn::hierarchical_control::AggravatedTask::update(){
        updateError();
        updateJacobian();
}
