#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V2_0_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V2_0_H

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

namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class SupportShapingV2
{

public:
  SupportShapingV2(mwoibn::robot_class::Robot& robot, YAML::Node config);

  ~SupportShapingV2(){}

mwoibn::Matrix& marginJ(){return _margins_jacobian;}
mwoibn::VectorN& margin(){return _margins;}

void init();
void update();

void solve();
void log(mwoibn::common::Logger& logger);
mwoibn::VectorN get(){return _optimal_state;}

void addTask(mwoibn::hierarchical_control::tasks::BasicTask& task){_other_tasks.push_back(&task);}

mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& points(){return _contact_points;}

protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::robot_points::Point& _base;

  double _trace, cost__;
  mwoibn::VectorN _safety, _workspace;
  mwoibn::Matrix _cost;
  mwoibn::VectorN _margins, _norms;
  mwoibn::Matrix _margins_jacobian;
  mwoibn::Matrix _equality_matrix, _inequality_matrix;
  mwoibn::VectorN _equality_vector, _inequality_vector;
  mwoibn::VectorN _optimal_state;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _contact_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _base_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _hips, _wheels;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _workspace_points;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>> _wheel_transforms;

  std::vector<std::pair<int,int>> _margin_pairs;
  Eigen::LLT<Eigen::MatrixXd,Eigen::Lower> _llt;

  std::vector<mwoibn::hierarchical_control::tasks::BasicTask*> _other_tasks;

  void _update();

  // I should check if it works before making the full optimization
  void _computeMargin(int i);

  // Do I have a way to validate it?
  // Try to integrate and compare with next values m + dt J \dot q = m_1+i
  double _marginJacobians();
  // AgumentedJacobian of previous tasks
  // limits? - position/velocity/ steerings - _robot.lower_limits/ _robot.upper_limits
  // extended_state - for now try to optimize over the robot state - but I will need the x/y desired to compute the steering
  // time_step - robot.rate
  // computeMargin
  // Margin _jacobian
  // Max workspace (this may be annoying to get from the urdf automatically - use config file for now?)


};
}
}
#endif
