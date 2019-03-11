#ifndef __MGNSS_HIGHER_LEVEL_QR_TRACKING_H
#define __MGNSS_HIGHER_LEVEL_QR_TRACKING_H

#include "eiquadprog/eiquadprog.hh"
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"

#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/handler.h"

#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/robot_points/linear_point.h"
#include "mwoibn/robot_points/minus.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/robot_points/ground_wheel.h"

#include "mwoibn/common/logger.h"

// #include "mgnss/higher_level/state_machine.h"
#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss
{

namespace higher_level
{

// class Limit;
/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class QrTracking
{

public:
  QrTracking(mwoibn::robot_class::Robot& robot, YAML::Node config, const mwoibn::VectorN& reference, const mwoibn::VectorN& current, double gain, std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace);

  ~QrTracking(){}

// mwoibn::Matrix& marginJ(){return _margins_jacobian;}
// mwoibn::VectorN& margin(){return _margins;}

void init();
void _update();

void solve();
void log(mwoibn::common::Logger& logger);
mwoibn::VectorN get(){return _return_state;}

// void addTask(mwoibn::hierarchical_control::tasks::BasicTask& task){_other_tasks.push_back(&task);}

// mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& points(){return _contact_points;}

protected:
  mwoibn::robot_class::Robot& _robot;

  int _size = 4;
  double _trace, _optimal_cost, _slack, _vars, _gain;
  mwoibn::VectorN _safety, _linear_cost, _vector_cost_, _desired, _max_workspace;
  mwoibn::Matrix _quadratic_cost;


  const mgnss::higher_level::Limit &_margin, &_workspace;
  // const mwoibn::VectorN &_margins, &_workspace;
  // const mwoibn::Matrix &_margins_jacobian, &_workspace_jacobian;
  mwoibn::Matrix _equality_matrix, _inequality_matrix;
  mwoibn::VectorN _equality_vector, _inequality_vector;
  mwoibn::VectorN _optimal_state, _return_state;
  const mwoibn::VectorN& _reference, &_current;
  mwoibn::VectorN _offset_margin, _offset_workspace;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& _wheel_transforms;

  Eigen::LLT<Eigen::MatrixXd,Eigen::Lower> _llt;



};
}
}
#endif
