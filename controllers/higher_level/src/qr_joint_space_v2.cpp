#include "mgnss/higher_level/qr_joint_space_v2.h"
// #include <range/v3/view/zip.hpp>
// #include <range/v3/view/concat.hpp>

mgnss::higher_level::QRJointSpaceV2::QRJointSpaceV2(mgnss::higher_level::QrTask& task, const mwoibn::Matrix& jacobian, const mwoibn::VectorN& offset, mwoibn::robot_class::Robot& robot, double damping):
    mgnss::higher_level::QrTask(), _robot(robot), _task(task), _jacobian(jacobian), _offset(offset), _damping(damping){

}

void mgnss::higher_level::QRJointSpaceV2::init(){
  _task.init();
  resize(_jacobian.cols(), _task.soft_inequality.rows());
  _damp_vec.setConstant(_vars, _damping);
  _slack = _task.soft_inequality.rows();

//  equality.clear();
//  soft_inequality.clear();
//  hard_inequality.clear();

  for (auto& constraint: _task.equality)
    equality.add(Constraint(constraint->size(), _vars));

  for (auto& constraint: _task.soft_inequality)
    soft_inequality.add(Constraint(constraint->size(), _vars));

  for (auto& constraint: _task.hard_inequality)
      hard_inequality.add(Constraint(constraint->size(), _vars));

  QrTask::init();

  _return_state.setZero(_jacobian.rows());
}

void mgnss::higher_level::QRJointSpaceV2::_update(){
      _task._update();
     // _marginJacobians();
     _optimal_state.setZero();// = _robot.command.velocity.get(); // init from last command?



     for(auto&& zip:  ranges::view::zip(_task.equality, ranges::view::slice(equality, ranges::end - _task.equality.size(), ranges::end) ) ){
       // std::cout << "equality\n" << std::get<0>(zip)->state.transpose() << "jacobian\n" << std::get<0>(zip)->jacobian << std::endl;
       int old_ = std::get<0>(zip)->jacobian.cols();
       int new_ = std::get<1>(zip)->jacobian.cols();
       int size_ = std::get<0>(zip)->state.size();

       std::get<1>(zip)->state = std::get<0>(zip)->state + std::get<0>(zip)->jacobian.leftCols(old_)*_offset;

       std::get<1>(zip)->jacobian.block(0,0,size_, new_) = std::get<0>(zip)->jacobian.leftCols(old_)*_jacobian.leftCols(new_);

       // std::cout << "equality\n" << std::get<1>(zip)->state.transpose() << "jacobian\n" << std::get<1>(zip)->jacobian << std::endl;
     }
     for(auto&& zip:  ranges::view::zip(_task.soft_inequality, ranges::view::slice(soft_inequality, ranges::end - _task.soft_inequality.size(), ranges::end)   )  ){
       // std::cout << "equality\n" << std::get<0>(zip)->state.transpose() << "jacobian\n" << std::get<0>(zip)->jacobian << std::endl;
       int old_ = std::get<0>(zip)->jacobian.cols();
       int new_ = std::get<1>(zip)->jacobian.cols();
       int size_ = std::get<0>(zip)->state.size();

       std::get<1>(zip)->state = std::get<0>(zip)->state + std::get<0>(zip)->jacobian.leftCols(old_)*_offset;

       std::get<1>(zip)->jacobian.block(0,0,size_, new_) = std::get<0>(zip)->jacobian.leftCols(old_)*_jacobian.leftCols(new_);

       // std::cout << "equality\n" << std::get<1>(zip)->state.transpose() << "jacobian\n" << std::get<1>(zip)->jacobian << std::endl;
     }
     for(auto&& zip:  ranges::view::zip(_task.hard_inequality, ranges::view::slice(hard_inequality, ranges::end - _task.hard_inequality.size(), ranges::end)) ){
       // std::cout << "equality\n" << std::get<0>(zip)->state.transpose() << "jacobian\n" << std::get<0>(zip)->jacobian << std::endl;
       int old_ = std::get<0>(zip)->jacobian.cols();
       int new_ = std::get<1>(zip)->jacobian.cols();
       int size_ = std::get<0>(zip)->state.size();

       std::get<1>(zip)->state = std::get<0>(zip)->state + std::get<0>(zip)->jacobian.leftCols(old_)*_offset;

       std::get<1>(zip)->jacobian.block(0,0,size_, new_) = std::get<0>(zip)->jacobian.leftCols(old_)*_jacobian.leftCols(new_);

       // std::cout << "equality\n" << std::get<1>(zip)->state.transpose() << "jacobian\n" << std::get<1>(zip)->jacobian << std::endl;
     }


     // for(auto&& zip: ranges::view::zip(_task.soft_inequality, soft_inequality)){
       // std::cout << "soft_inequality\n" << std::get<0>(zip)->state.transpose() << "soft_inequality\n" << std::get<0>(zip)->jacobian << std::endl;
     //   std::get<1>(zip)->state = std::get<0>(zip)->state + std::get<0>(zip)->jacobian.leftCols(old_)*_offset;
     //   std::get<1>(zip)->jacobian.block(0,0,size_, new_) = std::get<0>(zip)->jacobian.leftCols(old_)*_jacobian.leftCols(new_);
     //   // std::cout << "soft_inequality\n" << std::get<1>(zip)->state.transpose() << "soft_inequality\n" << std::get<1>(zip)->jacobian << std::endl;
     // }
     // for(auto&& zip: ranges::view::zip(_task.hard_inequality, hard_inequality)){
     //   // std::cout << "hard_inequality\n" << std::get<0>(zip)->state.transpose() << "hard_inequality\n" << std::get<0>(zip)->jacobian << std::endl;
     //   std::get<1>(zip)->state = std::get<0>(zip)->state + std::get<0>(zip)->jacobian.leftCols(old_)*_offset;
     //   std::get<1>(zip)->jacobian.block(0,0,size_, new_) = std::get<0>(zip)->jacobian.leftCols(old_)*_jacobian.leftCols(new_);
     //   // std::cout << "hard_inequality\n" << std::get<1>(zip)->state.transpose() << "hard_inequality\n" << std::get<1>(zip)->jacobian << std::endl;
     // }


          // _cost.quadratic.block(0,0,_vars, _vars).setIdentity();
          // _cost.quadratic.block(_vars,_vars, _task.slack(), _task.slack())  = _task.cost().quadratic.block(_task.vars(), _task.vars(), _task.slack(), _task.slack());
          // _cost.linear.head(_vars).setZero();
          // _cost.linear.tail(_task.slack())  = _task.cost().linear.tail(_task.slack());


  _cost.quadratic.block(0,0,_vars, _vars)  = _jacobian.transpose() * _task.cost().quadratic.block(0,0,_task.vars(), _task.vars()) * _jacobian;
  _cost.quadratic.block(0,0,_vars, _vars) += _damp_vec.asDiagonal();
  _cost.quadratic.block(_vars,_vars, _task.slack(), _task.slack())  = _task.cost().quadratic.block(_task.vars(), _task.vars(), _task.slack(), _task.slack());
  _cost.linear.head(_vars)  = _task.cost().linear.head(_task.vars()).transpose() * _jacobian;
  _cost.linear.head(_vars) += _offset.transpose()*_task.cost().quadratic.block(0,0,_task.vars(), _task.vars())*_jacobian;
  _cost.linear.tail(_task.slack())  = _task.cost().linear.tail(_task.slack());

  // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
  // std::cout << "_cost.linear\n" << _cost.linear.transpose() << std::endl;
  // std::cout << "_inequality.jacobian\n" << _inequality.jacobian << std::endl;
  // std::cout << "_inequality.state\n" << _inequality.state << std::endl;

  // std::cout << "_equality.jacobian\n" << _equality.jacobian << std::endl;

  QrTask::_update();
  // std::cout << "equality.transposed\n" << _equality.transposed << std::endl;
  // std::cout << "equality.jacobian\n" << _equality.jacobian << std::endl;
  //
  // std::cout << "soft_inequality\n" << soft_inequality.getJacobian() << std::endl;

// std::cout << "_jacobian\n" << _jacobian << std::endl;

}

void mgnss::higher_level::QRJointSpaceV2::log(mwoibn::common::Logger& logger){

  //   std::cout << "QR JOINT SPACE STATE" << std::endl;
  if(_return_state.norm() > mwoibn::EPS){
    std::cout << "_offset\t" << _offset.transpose() << std::endl;
    std::cout << "_jacobian\t" << _jacobian << std::endl;
    std::cout << "_optimal_state\t" << _optimal_state.transpose() << std::endl;
    std::cout << "_return_state\t" << _return_state.transpose() << std::endl;
    std::cout << "_cost.linear\t" << _cost.linear.transpose() << std::endl;
    std::cout << "_cost.quadratic\t" << _cost.quadratic.transpose() << std::endl;

    for(auto&& zip: ranges::view::zip(_task.equality, equality)){
      std::cout << "equality\n" << std::get<0>(zip)->state.transpose() << "jacobian\n" << std::get<0>(zip)->jacobian << std::endl;
      std::cout << "equality\n" << std::get<1>(zip)->state.transpose() << "jacobian\n" << std::get<1>(zip)->jacobian << std::endl;
    }
    for(auto&& zip: ranges::view::zip(_task.soft_inequality, soft_inequality)){
      std::cout << "soft_inequality\n" << std::get<0>(zip)->state.transpose() << "soft_inequality\n" << std::get<0>(zip)->jacobian << std::endl;
      std::cout << "soft_inequality\n" << std::get<1>(zip)->state.transpose() << "soft_inequality\n" << std::get<1>(zip)->jacobian << std::endl;
    }
    for(auto& constraint:  hard_inequality ){
      std::cout << "hard_inequality\n" << constraint->state.transpose()  << std::endl;
    }
  }
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

void mgnss::higher_level::QRJointSpaceV2::_outputTransform(){
  // std::cout << "_optimal_state.transpose" << _optimal_state.head(_vars).transpose() << std::endl;
  // std::cout << "_offset.transpose" << _offset.transpose() << std::endl;
  _return_state = _jacobian*_optimal_state.head(_vars) + _offset;



}
