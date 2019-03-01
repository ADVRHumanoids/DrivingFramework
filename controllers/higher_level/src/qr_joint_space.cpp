#include "mgnss/higher_level/qr_joint_space.h"
// #include <range/v3/view/zip.hpp>

mgnss::higher_level::QRJointSpace::QRJointSpace(mgnss::higher_level::QrTask& task, const mwoibn::Matrix& jacobian, mwoibn::robot_class::Robot& robot):
    mgnss::higher_level::QrTask(robot.getDofs(), task.soft_inequality.rows()), _robot(robot), _task(task), _jacobian(jacobian){

}

void mgnss::higher_level::QRJointSpace::init(){

  resize(_robot.getDofs(), _task.soft_inequality.rows());
  _slack = _task.soft_inequality.rows();

  for (auto& constraint: _task.equality)
    equality.add(Constraint(constraint->size(), _vars));

  for (auto& constraint: _task.soft_inequality)
    soft_inequality.add(Constraint(constraint->size(), _vars));

  for (auto& constraint: _task.hard_inequality)
      hard_inequality.add(Constraint(constraint->size(), _vars));

  QrTask::init();
}

void mgnss::higher_level::QRJointSpace::_update(){
      _task._update();
     // _marginJacobians();
     _optimal_state.setZero();// = _robot.command.velocity.get(); // init from last command?

     for(auto&& zip: ranges::view::zip(_task.equality, equality)){
       std::get<1>(zip)->state = std::get<0>(zip)->state;
       std::get<1>(zip)->jacobian.noalias() = std::get<0>(zip)->jacobian*_jacobian;
     }
     for(auto&& zip: ranges::view::zip(_task.soft_inequality, soft_inequality)){
       std::get<1>(zip)->state = std::get<0>(zip)->state;
       std::get<1>(zip)->jacobian.noalias() = std::get<0>(zip)->jacobian*_jacobian;
     }
     for(auto&& zip: ranges::view::zip(_task.hard_inequality, hard_inequality)){
       std::get<1>(zip)->state = std::get<0>(zip)->state;
       std::get<1>(zip)->jacobian.noalias() = std::get<0>(zip)->jacobian*_jacobian;
     }

  _cost.quadratic.block(0,0,_vars, _vars)  = _jacobian.transpose() * _task.cost().quadratic.block(0,0,_task.vars(), _task.vars()) * _jacobian;
  _cost.quadratic.block(0,0,_vars, _vars) += mwoibn::VectorN::Constant(_vars, 1e-8).asDiagonal();
  _cost.quadratic.block(_vars,_vars, _task.slack(), _task.slack())  = _task.cost().quadratic.block(_task.vars(), _task.vars(), _task.slack(), _task.slack());
  _cost.linear.head(_vars)  = _task.cost().linear.head(_task.vars()).transpose() * _jacobian;
  _cost.linear.tail(_task.slack())  = _task.cost().linear.tail(_task.slack());

  // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
  std::cout << "_cost.linear\n" << _cost.linear.transpose() << std::endl;
  // std::cout << "_inequality.jacobian\n" << _inequality.jacobian << std::endl;
  // std::cout << "_inequality.state\n" << _inequality.state << std::endl;

  // std::cout << "_equality.jacobian\n" << _equality.jacobian << std::endl;

  QrTask::_update();
  // std::cout << "equality.transposed\n" << _equality.transposed << std::endl;
  // std::cout << "equality.jacobian\n" << _equality.jacobian << std::endl;
  //
  // std::cout << "soft_inequality\n" << soft_inequality.getJacobian() << std::endl;
}

void mgnss::higher_level::QRJointSpace::log(mwoibn::common::Logger& logger){
    // std::cout << "_optimal_state\t" << _optimal_state.transpose() << std::endl;

  if(_return_state.norm()){
    std::cout << "_optimal_state yaws\t";// << _return_state.transpose() << std::endl;
    for(auto& dof: mwoibn::eigen_utils::toVector<mwoibn::Scalar>(_robot.getDof(_robot.getLinks("yaws") ) ) )
     std::cout  << _return_state[dof] << " ";
     std::cout << std::endl;
   }

  // std::cout << "inequality\n" << _inequality.jacobian << std::endl;

    logger.add("cost", cost__);
    //
    // for (int i = 0; i < _vars; i++){
    //    logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);
    //    logger.add("final_cp_" + std::to_string(i), _return_state[i]);
    //  }
    //
    // for (int i = 0; i < _slack; i++)
    //    logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

}
