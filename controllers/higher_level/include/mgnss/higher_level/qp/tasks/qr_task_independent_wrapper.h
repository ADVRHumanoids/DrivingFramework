#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_INDEPENDENT_WRAPPER_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_INDEPENDENT_WRAPPER_H

// #include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
//#include "mgnss/higher_level/qp/tasks/qr_joint_space_v3.h"
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
class QrTaskIndependentWrapper: public QRJointSpaceV2
{

public:
  QrTaskIndependentWrapper(mwoibn::hierarchical_control::tasks::BasicTask& task, double gain, double damping, double damping_treshhold, mwoibn::robot_class::Robot& robot):
      QRJointSpaceV2(_qr_base, task.getJacobian(), _task_state, robot, damping), _qr_base(task.getTaskSize(),0), _basic_task(task){
    _gain.setConstant(_task.getTaskSize(), gain);
    _damping_max = damping;
    _damping_treshhold = damping_treshhold;
    resize(_robot.getDofs(),0);
  }

  QrTaskIndependentWrapper(mwoibn::hierarchical_control::tasks::BasicTask& task, const mwoibn::VectorN& gain, double damping, double damping_treshhold, mwoibn::robot_class::Robot& robot):
      QRJointSpaceV2(_qr_base, task.getJacobian(), _task_state, robot, damping), _qr_base(task.getTaskSize(),0), _basic_task(task), _gain(gain){
    resize(_robot.getDofs(),0);
    _damping_max = damping;
    _damping_treshhold = damping_treshhold;
  }

  virtual void resize(int vars, int slack){
    _qr_base.resize(_basic_task.getTaskSize(),0);

    QRJointSpaceV2::resize(vars, slack);

  }


virtual void _update(){
      _basic_task.update();

      _task_state = _gain.cwiseProduct(_basic_task.getError()) + _basic_task.getVelocity();
      for(int i = 0; i < _basic_task.getTaskSize(); i++){
        double min = _basic_task.getJacobian().row(i).cwiseAbs().min();
        if(min < _damping_treshhold)
          _damp_vec[i] = (1 - (min/_damping_treshhold)*(min/_damping_treshhold))*_damping_max;
        else _damp_vec = 0;
      }



      QRJointSpaceV2::_update();

}

protected:
  mwoibn::hierarchical_control::tasks::BasicTask& _basic_task;
  mwoibn::VectorN _task_state;
  mgnss::higher_level::QrTask _qr_base;
  mwoibn::VectorN _gain;
  double _damping_max, _damping_treshhold;

  virtual void _outputTransform(){


  }


};

}
}
#endif
