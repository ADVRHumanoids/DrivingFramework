#include "mgnss/higher_level/merge_manager.h"

mgnss::hierarchical_control::higher_level::MergeManager::MergeManager(){

}

void mgnss::hierarchical_control::higher_level::MergeManager::addTask(mwoibn::hierarchical_control::tasks::Merge task, double eps = 1e-8){

        if(_controller.getCommand().size() != task.getTaskDofs())
                throw(std::invalid_argument("Couldn't intialize the task, size is incompatibile with controller."));

        _tasks.push_back(std::make_pair(task, eps));
}

void mgnss::hierarchical_control::higher_level::MergeManager::initialize(){
        int size = 0;
        for(auto& task : _tasks)
                size += task.getTaskSize();

        mwoibn::VectorBool id(size);
        for (int i = 0; i < std::pow(2,size); i++) {
                boost::dynamic_bitset<> b(size, i);
                for(int j = 0; j < id.size(); j++)
                        id[j] = b[j];
                _support_tasks.insert(std::make_pair(id, BasicTask(size, _controller.getCommand().size())));
        }

        _active_actions.reserve(_support_tasks.size());
}

void mgnss::hierarchical_control::higher_level::MergeManager::update(){

        // double row = 0;
        // for(int i = 0; i < _tasks.size(); i++) {
        //         bool active = _tasks[i].det() < _eps ? false : true;
        //         for(int k = 0; k < _task.getTaskSize(); k++) {
        //                 _current_id.[row] = active;
        //                 ++row;
        //         }
        // }

        // check if the task is already running - and if anything changed
        // initalize a new task if needed then, check which tasks are active, then update proper jacobians, for new task update the last Jacobian too, or at least leave a comment its missing
        // it should store replacement tasks
        // for all active tasks update Jacobians
        // how to store active tasks
        // this can preallocate a specific amount of actions in the controller and store pointers to them? - or action can return the pointer to active task - this way I should have access to the API

        // _support_tasks[i]
}
