#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H

#include "mgnss/higher_level/support_shaping_v3_0.h"
#include "mgnss/higher_level/constraint.h"

#include "mwoibn/robot_class/robot.h"
// #include "mwoibn/hierarchical_control/tasks/controller_task.h"

#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/handler.h"

#include "mwoibn/dynamic_points/torus.h"

#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/robot_points/linear_point.h"
#include "mwoibn/robot_points/frame_orientation.h"
#include "mwoibn/robot_points/minus.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/robot_points/ground_wheel.h"


namespace mgnss
{

namespace higher_level
{



/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class StateMachine
{

public:
//StateMachine(mwoibn::robot_class::Robot& robot, YAML::Node config):
  StateMachine(mwoibn::robot_class::Robot& robot, YAML::Node config);

  ~StateMachine(){}

  void init();
  void update();

  // bool state(){ return _state; }


  // bool restart() { return _restart;}

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steeringFrames(){return _wheel_transforms;}
  std::vector<mwoibn::Matrix3> desiredSteer;
  //
  // const mwoibn::VectorN& margins(){return _margins;}
  // const mwoibn::VectorN& workspace(){return _workspace;}
  //
  // const mwoibn::Matrix& marginsJacobian(){return _margins_jacobian;}
  // const mwoibn::Matrix& workspaceJacobian(){return _workspace_jacobian;}
  const Limit& margin(){return _margins;}
  const Limit& workspace(){return _workspace;}

  const mwoibn::Matrix& stateJacobian(){return _state_jacobian;}
  const mwoibn::Matrix& worldJacobian(){return _world_jacobian;}
  // const mwoibn::Matrix& steerJacobian(){return _steer_jacobian;}
  // const mwoibn::Matrix& desiredJacobian(){return _desired_jacobian;}
  const mwoibn::VectorN& stateOffset(){return _state_offset;}
  // const mwoibn::VectorN& nextStateOffset(){return _next_state_offset;}

  mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus>& accelerations(){return _torus_acceleration;}
  const mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& wheelOrientation(){return _wheel_orientation;}

  void log(mwoibn::common::Logger& logger);
  // bool valid();

  // const mwoibn::VectorN& marginsLimits(){return _safety_margins;}
  // const mwoibn::VectorN& workspaceLimits(){return _max_workspace;}
  //
  // const mwoibn::VectorN& marginSafety();
  // const mwoibn::VectorN& workspaceSafety();

protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::robot_points::Point& _base;

  unsigned int _size;
  // mgnss::higher_level::SupportShapingV3& _shape;
  // mgnss::higher_level::QrTracking& _restore;

  // bool _state, _restart;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _wheel_orientation;

  mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> _contact_points;
  mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus> _torus_acceleration;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _base_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _hips, _wheels;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _workspace_points;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>> _wheel_transforms;

  std::vector<std::pair<int,int>> _margin_pairs;

  Limit _margins, _workspace;

  mwoibn::VectorN _norms, _state_offset;//, _next_state_offset;
  mwoibn::Matrix _state_jacobian, _world_jacobian, _steer_jacobian;//, _desired_jacobian;

  mwoibn::Matrix3 _support_jacobian;
  mwoibn::Vector3 _support_offset;

  void _computeMargin(int i);
  void _marginJacobians();

  void _computeWorkspace();
  void _workspaceJacobian();

  void _update();




};
}
}
#endif
