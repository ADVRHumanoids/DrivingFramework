#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_III_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_III_H


#include "mgnss/higher_level/qp/constraints/constraint.h"

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
#include "mgnss/higher_level/state_machine.h"


namespace mgnss
{

namespace higher_level
{

  class SupportStateII: public StateTransformation{
    public:
      SupportStateII(mwoibn::robot_class::Robot& robot): _robot(robot){}
      virtual void update();
      mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus> torus_acceleration;

    protected:
      mwoibn::robot_class::Robot& _robot;
      mwoibn::Matrix3 _support_jacobian, mat_1;
      mwoibn::Vector3 _support_offset, vec_1;

  };


/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class StateMachineIII
{

public:
  StateMachineIII(mwoibn::robot_class::Robot& robot, YAML::Node config);

  void init();
  virtual void update();


  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steeringFrames(){return _wheel_transforms;}
  // std::vector<mwoibn::Matrix3> desiredSteer;

  const Limit& margin(){return _margins;}
  const Limit& workspace(){return _workspace;}

  // const mwoibn::Matrix& stateJacobian(){return _state_jacobian;}
  // const mwoibn::Matrix& worldJacobian(){return _world_jacobian;}

  // const mwoibn::VectorN& stateOffset(){return _state_offset;}
  SupportStateII cost_I;
  StateTransformation cost_II;
  StateTransformation state_I, state_II;

  // mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus>& accelerations(){return _torus_acceleration;}
  // const mwoibn::robot_points::Handler<mwoibn::robot_points::Point>& wheelOrientation(){return _wheel_orientation;}

  void log(mwoibn::common::Logger& logger);


protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::robot_points::Point& _base;

  unsigned int _size;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _wheel_orientation;

  mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> _contact_points;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _base_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _hips, _wheels;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _workspace_points;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>> _wheel_transforms;

  std::vector<std::pair<int,int>> _margin_pairs;

  Limit _margins, _workspace;

  mwoibn::VectorN _norms;//, _state_offset;//, _next_state_offset;
  // mwoibn::Matrix _state_jacobian;//, _desired_jacobian;


  void _computeMargin(int i);
  virtual void _marginJacobians();

  void _computeWorkspace();
  virtual void _workspaceJacobian();

  void _update();

};
}
}
#endif
