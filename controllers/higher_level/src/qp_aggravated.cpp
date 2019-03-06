#include "mgnss/higher_level/qp_aggravated.h"
// #include <range/v3/view/zip.hpp>
// #include <range/v3/view/concat.hpp>

void mgnss::higher_level::QpAggravated::init(){

  _slack = 0;
  // for(auto& task: _tasks) task->init();

  for(auto& task: _tasks) _slack += task->soft_inequality.rows();

  resize(_vars, _slack);
//  equality.clear();
//  soft_inequality.clear();
//  hard_inequality.clear();


  for(auto& task: _tasks) {
    if(task->equality.size()){

      if(task->equality.cols()!= _vars)
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile equality constraint sizes got ") + std::to_string(task->equality.cols()) + " , expected " + std::to_string(_vars));
      equality.add(Constraint(task->equality.rows(), _vars));
    }

    if(task->soft_inequality.size()){

      if(task->soft_inequality.cols()!= _vars)
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile soft_inequality constraint sizes got ") + std::to_string(task->soft_inequality.cols()) + " , expected " + std::to_string(_vars));

      soft_inequality.add(Constraint(task->soft_inequality.rows(), _vars));
    }

    if(task->hard_inequality.size()){

      if(task->hard_inequality.cols()!= _vars)
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile hard_inequality constraint sizes got ") + std::to_string(task->hard_inequality.cols()) + " , expected " + std::to_string(_vars));

        hard_inequality.add(Constraint(task->hard_inequality.rows(), _vars));
      }
    }

  QrTask::init();

  // _return_state.setZero(_jacobian.rows());
}

void mgnss::higher_level::QpAggravated::_update(){


  _optimal_state.setZero();// = _robot.command.velocity.get(); // init from last command?

  for(auto& task: _tasks)  task->_update();

  int counter = 0;
  RANGES_FOR(auto&& task, ranges::view::reverse(_tasks) ){
    //int idx = _tasks.size() - 1 - std::get<0>(zip);
    // auto task = std::get<1>(zip);

      if(task->equality.size()){
       equality.end(counter).state = task->equality.getState();
       equality.end(counter).jacobian = task->equality.getJacobian();
       ++counter;
       // std::cout << "equality " << idx << std::endl;
       // std::cout << "state\t" << equality[idx].state.transpose() << std::endl;
       // std::cout << "jacobian\n" << equality[idx].jacobian << std::endl;

     }
  }

  counter = 0;
  RANGES_FOR(auto&& task, ranges::view::reverse(_tasks) ){

     if(task->soft_inequality.size()){
       soft_inequality.end(counter).state = task->soft_inequality.getState();
       soft_inequality.end(counter).jacobian = task->soft_inequality.getJacobian();
       ++counter;
       // std::cout << "soft_inequality " << idx << std::endl;
       // std::cout << "state\t" << soft_inequality[idx].state.transpose() << std::endl;
       // std::cout << "jacobian\n" << soft_inequality[idx].jacobian << std::endl;

     }
   }

   counter = 0;
   RANGES_FOR(auto&& task, ranges::view::reverse(_tasks) ){

     if(task->hard_inequality.size()){
       hard_inequality.end(counter).state = task->hard_inequality.getState();
       hard_inequality.end(counter).jacobian = task->hard_inequality.getJacobian();
       ++counter;
     }
   }

   _cost.quadratic.setZero();
   _cost.linear.setZero();
   int slack = 0;
   for(auto& task: _tasks){

     _cost.quadratic.block(0,0,_vars, _vars)  += task->cost().quadratic.block(0,0,_vars, _vars);
     _cost.quadratic.block(_vars+slack,_vars+slack, task->slack(), task->slack())  = task->cost().quadratic.block(task->vars(), task->vars(), task->slack(), task->slack());

     _cost.linear.head(_vars) += task->cost().linear.head(task->vars());
     _cost.linear.segment(_vars+slack, task->slack())  = task->cost().linear.tail(task->slack());

     // std::cout << "task cost\n" << task->cost().linear.transpose() << std::endl;
     // std::cout << "task cost\n"<< task->cost().quadratic << std::endl;

     slack += task->slack();
  }

  // std::cout << "aggravated cost\n" << _cost.linear.transpose() << std::endl;
  // std::cout << "aggravated cost\n" <<  _cost.quadratic << std::endl;


  QrTask::_update();

}

void mgnss::higher_level::QpAggravated::log(mwoibn::common::Logger& logger){

    logger.add("cost", cost__);

}
