#include "mwoibn/hierarchical_control/tasks/aggravated.h"

//  //   add task at the end of the stack
void mwoibn::hierarchical_control::tasks::Aggravated::addTask(BasicTask& task){
        mwoibn::VectorBool selector;
        selector.setConstant(task.getTaskSize(), true);

        _verify(task, selector);

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        mwoibn::VectorN test__ = mwoibn::VectorN::Constant(task.getTaskSize(), 1);
        _tasks_ptrs.push_back(std::make_tuple(selector, std::ref(task), test__));

}

void mwoibn::hierarchical_control::tasks::Aggravated::addTask(BasicTask& task, mwoibn::VectorBool selector){

        _verify(task, selector);

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        mwoibn::VectorN test__ = mwoibn::VectorN::Constant(task.getTaskSize(), 1);
        _tasks_ptrs.push_back(std::make_tuple(selector, std::ref(task), test__));
}
// add task at the end of the stack
void mwoibn::hierarchical_control::tasks::Aggravated::addTask(BasicTask& task, unsigned int i){
        mwoibn::VectorBool selector;
        selector.setConstant(task.getTaskSize(), true);
        _verify(task, selector);

        if(i > _tasks_ptrs.size())
                throw(std::invalid_argument("Couldn't add task to the aggravated task, requested number is too high."));

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        mwoibn::VectorN test__ = mwoibn::VectorN::Constant(task.getTaskSize(), 1);
        _tasks_ptrs.insert(_tasks_ptrs.begin()+i, std::make_tuple(selector, std::ref(task), test__));

}

//  // i - zero based
void mwoibn::hierarchical_control::tasks::Aggravated::addTask(BasicTask& task, mwoibn::VectorBool selector, unsigned int i){
        _verify(task, selector);

        if(i > _tasks_ptrs.size())
                throw(std::invalid_argument("Couldn't add task to the aggravated task, requested number is too high."));

        _init(getTaskSize()+selector.count(), task.getTaskDofs());
        mwoibn::VectorN test__ = mwoibn::VectorN::Constant(task.getTaskSize(), 1);

        _tasks_ptrs.insert(_tasks_ptrs.begin()+i, std::make_tuple(selector, std::ref(task), test__));
}

void mwoibn::hierarchical_control::tasks::Aggravated::updateJacobian(){
        _last_jacobian.noalias() = _jacobian;
        int row = 0;
        for(auto& task : _tasks_ptrs) {
                std::get<1>(task).updateJacobian();
                for(int i = 0; i < std::get<0>(task).size(); i++) {
                        if(!std::get<0>(task)[i]) continue;
                        _jacobian.row(row) = std::get<2>(task)[i]*std::get<1>(task).getJacobian().row(i);
                        row++;
                }
        }
}
//! generic function to provide the same syntax for error update of all
//derived classes
void mwoibn::hierarchical_control::tasks::Aggravated::updateError(){

        _last_error.noalias() = _error;
        int row = 0;
        for(auto& task : _tasks_ptrs) {
                std::get<1>(task).updateError();
                for(int i = 0; i < std::get<0>(task).size(); i++) {
                        if(!std::get<0>(task)[i]) continue;
                        _error[row] = std::get<2>(task)[i]*std::get<1>(task).getError()[i];
                        _velocity[row] = std::get<2>(task)[i]*std::get<1>(task).getVelocity()[i];
                        row++;
                }
        }
}
//! updates whole task in one call, calls updateError() and updateJacobin() in
//that order

void mwoibn::hierarchical_control::tasks::Aggravated::update(){
        updateError();
        updateJacobian();
}
