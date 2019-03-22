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
    resize(_robot.getDofs(),0);
  }

  virtual void resize(int vars, int slack){
    _qr_base.resize(_basic_task.getTaskSize(),0);
    QRJointSpaceV2::resize(vars, slack);

  }


virtual void _update(){
      _basic_task.update();

      _task_state = _gain.cwiseProduct(_basic_task.getError()) + _basic_task.getVelocity();
      QRJointSpaceV2::_update();

}

protected:
  mwoibn::hierarchical_control::tasks::BasicTask& _basic_task;
  mwoibn::VectorN _task_state;
  mgnss::higher_level::QrTask _qr_base;
  mwoibn::VectorN _gain;

  virtual void _outputTransform(){


  }


};

}
}
#endif
