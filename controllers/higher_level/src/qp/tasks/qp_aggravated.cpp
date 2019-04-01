#include "mgnss/higher_level/qp/tasks/qp_aggravated.h"
// #include <range/v3/view/zip.hpp>
// #include <range/v3/view/concat.hpp>

void mgnss::higher_level::QpAggravated::init(){

  _slack = soft_inequality.rows();
  // for(auto& task: _tasks) task->init();

  for(auto& task: _tasks) _slack += task->soft_inequality.rows();

  resize(_vars, _slack);
//  equality.clear();
//  soft_inequality.clear();
//  hard_inequality.clear();


  for(auto& task: _tasks) {
    if(task->equality.size()){

      if(task->equality.cols() < _max )
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile equality constraint sizes got ") + std::to_string(task->equality.cols()) + " , expected at least " + std::to_string(_max));
      equality.add(Constraint(task->equality.rows(), task->equality.cols()));
    }

    if(task->soft_inequality.size()){

      if(task->soft_inequality.cols()  < _max )
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile soft_inequality constraint sizes got ") + std::to_string(task->soft_inequality.cols()) + " , expected at least " + std::to_string(_max));
        _updateGains();
      addSoft(Constraint(task->soft_inequality.rows(), task->soft_inequality.cols()), task->getSoftGain());
    }

    if(task->hard_inequality.size()){

      if(task->hard_inequality.cols()  < _max )
        throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(": incompatibile hard_inequality constraint sizes got ") + std::to_string(task->hard_inequality.cols()) + " , expected at least " + std::to_string(_max));

        hard_inequality.add(Constraint(task->hard_inequality.rows(), task->hard_inequality.cols()));
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
       equality.end(counter).setState() = task->equality.getState();
       for(int i = 0; i < _vars; i++)
          equality.end(counter).setJacobian().col(i) = task->equality.getJacobian().col(_chain[i]);
       ++counter;
       // std::cout << "equality " << idx << std::endl;
       // std::cout << "state\t" << equality[idx].state.transpose() << std::endl;
       // std::cout << "jacobian\n" << equality[idx].jacobian << std::endl;

     }
  }

  counter = 0;
  RANGES_FOR(auto&& task, ranges::view::reverse(_tasks) ){

     if(task->soft_inequality.size()){
       soft_inequality.end(counter).setState() = task->soft_inequality.getState();
       for(int i = 0; i < _vars; i++)
          soft_inequality.end(counter).setJacobian().col(i) = task->soft_inequality.getJacobian().col(_chain[i]);
       soft_inequality.end(counter).setGain(task->getSoftGain());
       _updateGains();
       ++counter;
       // std::cout << "soft_inequality " << idx << std::endl;
       // std::cout << "state\t" << soft_inequality[idx].state.transpose() << std::endl;
       // std::cout << "jacobian\n" << soft_inequality[idx].jacobian << std::endl;

     }
   }

   counter = 0;
   RANGES_FOR(auto&& task, ranges::view::reverse(_tasks) ){

     if(task->hard_inequality.size()){
       hard_inequality.end(counter).setState() = task->hard_inequality.getState();
       for(int i = 0; i < _vars; i++)
          hard_inequality.end(counter).setJacobian().col(i) = task->hard_inequality.getJacobian().col(_chain[i]);
       ++counter;
     }
   }

   _cost.quadratic.setZero();
   _cost.linear.setZero();
   int slack = 0;
   for(auto& task: _tasks){


      for(int i = 0; i < _vars; i++){
        _cost.linear[i] += task->cost().linear[_chain[i]];
         for(int j = 0; j < _vars; j++)
            _cost.quadratic(i,j)  += task->cost().quadratic(_chain[i],_chain[j]);
      }

     _cost.quadratic.block(_vars+slack,_vars+slack, task->slack(), task->slack())  = task->cost().quadratic.block(task->vars(), task->vars(), task->slack(), task->slack());



     _cost.linear.segment(_vars+slack, task->slack())  = task->cost().linear.tail(task->slack());

     // std::cout << "task cost\n" << task->cost().linear.transpose() << std::endl;
     // std::cout << "task cost\n"<< task->cost().quadratic << std::endl;

     slack += task->slack();
  }

  // std::cout << "aggravated cost\n" << _cost.linear.transpose() << std::endl;
  // std::cout << "aggravated cost\n" <<  _cost.quadratic << std::endl;


  QrTask::_update();
  // std::cout << "soft_gains\t" << _soft_gains.transpose() << std::endl;

}

void mgnss::higher_level::QpAggravated::log(mwoibn::common::Logger& logger){

    logger.add("cost", _optimal_cost);

}
