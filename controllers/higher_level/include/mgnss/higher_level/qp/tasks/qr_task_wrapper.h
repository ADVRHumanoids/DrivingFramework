#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_WRAPPER_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_WRAPPER_H

// #include "eiquadprog/eiquadprog.hh" //?
// #include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mgnss/higher_level/qp/tasks/qr_joint_space_v2.h"
// #include "mwoibn/robot_points/handler.h"
// #include "mwoibn/common/logger.h"
// #include "mgnss/higher_level/qp/constraints/constraint.h"


namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class QrTaskWrapper: public QRJointSpaceV2
{

public:
  QrTaskWrapper(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, mwoibn::robot_class::Robot& robot):
      QRJointSpaceV2(_qr_base, task.getJacobian(), _task_state, robot, damping), _qr_base(task.getTaskSize(),0), _basic_task(task){
    _gain.setConstant(_task.getTaskSize(), gain);
    resize(_robot.getDofs(),0);
  }

  QrTaskWrapper(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gain, double damping, mwoibn::robot_class::Robot& robot):
      QRJointSpaceV2(_qr_base, task.getJacobian(), _task_state, robot, damping), _qr_base(task.getTaskSize(),0), _basic_task(task), _gain(gain){
    // _gain.setConstant(_task.getTaskSize(), gain);
    resize(_robot.getDofs(),0);
  }

  virtual void resize(int vars, int slack){
    _qr_base.resize(_basic_task.getTaskSize(),0);
    // _gain.setConstant(_basic_task.getTaskSize(), _gain[0]);
    QRJointSpaceV2::resize(vars, slack);
    // std::cout << "_damping\t" << _damping << std::endl;

  }


virtual void _update(){
      _basic_task.update();
      // std::cout << "_gain\t" << _gain.transpose() << std::endl;
      // std::cout << "_basic_task\t" << _basic_task.getError().transpose() << std::endl;

      _task_state = _gain.cwiseProduct(_basic_task.getError()) + _basic_task.getVelocity();
      QRJointSpaceV2::_update();


      // std::cout << "_task_state\t" << _task_state.transpose() << std::endl;
      // std::cout << "_equality\t" << _equality.state.size() << std::endl;

      // _cost.quadratic.setIdentity();
      // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
      // std::cout << "_cost.linear\n" << _cost.linear.transpose() << std::endl;
}

protected:
  mwoibn::hierarchical_control::tasks::BasicTask& _basic_task;
  mwoibn::VectorN _task_state;
  mgnss::higher_level::QrTask _qr_base;
  mwoibn::VectorN _gain;

  virtual void _outputTransform(){

    // std::cout << "_optimal_state\t" << ( _optimal_state.head(_vars)).transpose()<< std::endl;

    // std::cout << ( _jacobian*_optimal_state.head(_vars) + _offset ).transpose()<< std::endl;

    // if(_return_state.norm() > mwoibn::EPS){
    //   std::cout << "_offset\t" << _offset.transpose() << std::endl;
    //   std::cout << "_jacobian\t" << _jacobian << std::endl;
    //   std::cout << "_optimal_state\t" << _optimal_state.transpose() << std::endl;
    //   std::cout << "_return_state\t" << _return_state.transpose() << std::endl;
    //   // std::cout << "_cost.linear\t" << _cost.linear.transpose() << std::endl;
    //   // std::cout << "_cost.quadratic\t" << _cost.quadratic.transpose() << std::endl;
    //   std::cout << "_equality\t" << _equality.jacobian << std::endl;
    //
    //   for(auto&& zip: ranges::view::zip(_task.equality, equality)){
    //     std::cout << "equality\n" << std::get<0>(zip)->state.transpose() << "jacobian\n" << std::get<0>(zip)->jacobian << std::endl;
    //     std::cout << "equality\n" << std::get<1>(zip)->state.transpose() << "jacobian\n" << std::get<1>(zip)->jacobian << std::endl;
    //   }
    //   for(auto&& zip: ranges::view::zip(_task.soft_inequality, soft_inequality)){
    //     std::cout << "soft_inequality\n" << std::get<0>(zip)->state.transpose() << "soft_inequality\n" << std::get<0>(zip)->jacobian << std::endl;
    //     std::cout << "soft_inequality\n" << std::get<1>(zip)->state.transpose() << "soft_inequality\n" << std::get<1>(zip)->jacobian << std::endl;
    //   }
    //   for(auto& constraint:  hard_inequality ){
    //     std::cout << "hard_inequality\n" << constraint->state.transpose()  << std::endl;
    //   }
    // }

  }


};

}
}
#endif
