#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_H

#include "eiquadprog/eiquadprog.hh" //?
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/common/logger.h"
#include "mgnss/higher_level/qp/constraints/constraint.h"


namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class QrTask: public mwoibn::hierarchical_control::tasks::BasicTask
{

public:
  QrTask(int vars, int slack);
  QrTask();

  ~QrTask(){  }


virtual void init();
virtual void resize(int vars, int slack);
virtual void clear();
virtual void _update();

virtual void solve();
virtual void log(mwoibn::common::Logger& logger){}
virtual const mwoibn::VectorN& get(){return _return_state;}
virtual const mwoibn::VectorN& raw(){return _optimal_state;}

double vars(){return _vars;}
double slack(){return _slack;}

const Cost& cost(){return _cost;}
// const Constraint& inequality(){return _inequality;}

mwoibn::robot_points::Handler<Constraint> equality, soft_inequality, hard_inequality;

//
// void addTask(mwoibn::hierarchical_control::tasks::BasicTask& task){_other_tasks.push_back(&task);}
//
// mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& points(){return _contact_points;}

protected:
  int _vars, _slack;
  double _trace, cost__;

  Cost _cost;

  Constraint _inequality, _equality;

  mwoibn::VectorN _optimal_state, _return_state;

  Eigen::LLT<Eigen::MatrixXd,Eigen::Lower> _llt;

  virtual void _outputTransform(){}

};

}
}
#endif
