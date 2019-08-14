#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_QR_TASK_H

#include "eiquadprog/eiquadprog_RT.hh" //?
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/common/logger.h"
#include "mgnss/higher_level/qp/constraints/constraint.h"
#include "mgnss/higher_level/qp/constraints/soft_constraint.h"


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
const mwoibn::VectorN& getSoftGain(){return _soft_gains;}

double vars(){return _vars;}
double slack(){return _slack;}

Cost& cost(){return _cost;}
double optimalCost(){return _optimal_cost;}
// const Constraint& inequality(){return _inequality;}


template<typename ConstraintType, typename GainType>
void addSoft(ConstraintType&& constraint, GainType gain){
    soft_inequality.add(constraints::SoftConstraint(std::move(constraint), gain));
    resize(_vars, soft_inequality.rows());
}

template<typename ConstraintType, typename GainType>
void addSoft(std::unique_ptr<ConstraintType> constraint, GainType gain){
    soft_inequality.add(constraints::SoftConstraint(std::move(constraint), gain));
    resize(_vars, soft_inequality.rows());
}

mwoibn::robot_points::Handler<Constraint> equality, hard_inequality;
mwoibn::robot_points::Handler<constraints::SoftConstraint> soft_inequality;

void set(const mwoibn::VectorN& state){
  _optimal_state.head(_vars) = state.head(_vars);
}

void transform(){
  _outputTransform();
}

bool verbose = false;

// void addTask(mwoibn::hierarchical_control::tasks::BasicTask& task){_other_tasks.push_back(&task);}
//
// mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& points(){return _contact_points;}

protected:
  int _vars, _slack;
  double _trace, _optimal_cost;

  Cost _cost;

  Constraint _inequality, _equality;

  mwoibn::VectorN _optimal_state, _return_state;

  Eigen::LLT<Eigen::MatrixXd,Eigen::Lower> _llt;

  Eigen::QuadProg _solver;
  virtual void _outputTransform(){}

  mwoibn::VectorN _soft_gains;

  void _updateGains(){
    int size = 0;
    for(auto& constraint: soft_inequality){
      _soft_gains.segment(size, constraint->rows()) = constraint->getGain();
      size += constraint->rows();
    }
  }

};

}
}
#endif
