#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

#include "mwoibn/robot_class/robot.h"
// #include "mwoibn/hierarchical_control/tasks/controller_task.h"

#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/handler.h"

#include "mwoibn/dynamic_points/torus.h"
#include "mwoibn/dynamic_points/torus2.h"
#include "mwoibn/dynamic_points/acceleration.h"
// #include "mwoibn/point_handling/linear_acceleration.h"
// #include "mwoibn/point_handling/linear_velocity.h"
#include "mwoibn/dynamic_points/torus_roll.h"
#include "mwoibn/dynamic_points/torus_integrated_roll.h"

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

class StateOffset{
  public:
    const mwoibn::VectorN& get(){return _offset; }
    mwoibn::VectorN& set(){return _offset; }

  protected:
    mwoibn::VectorN _offset;
};

class StateJacobian{

  public:
    const mwoibn::Matrix& get(){return _jacobian; }
    mwoibn::Matrix& set(){return _jacobian; }

  protected:
    mwoibn::Matrix _jacobian;
};

class StateTransformation{
  public:
    StateOffset offset;
    StateJacobian jacobian;
    virtual void update(){};
};

class SupportState: public StateTransformation{
  public:
    SupportState(mwoibn::robot_class::Robot& robot): _robot(robot){}
    virtual void update();
    mwoibn::robot_points::Handler<mwoibn::dynamic_points::TorusIntegratedRoll> torus_acceleration;

  protected:
    mwoibn::robot_class::Robot& _robot;
    mwoibn::Matrix3 _support_jacobian, mat_1;
    mwoibn::Vector3 _support_offset, vec_1;

};

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
  virtual void update();


  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steeringFrames(){return _wheel_transforms;}
  // std::vector<mwoibn::Matrix3> desiredSteer;

  const Limit& margin(){return _margins;}
  const Limit& workspace(){return _workspace;}

  // const mwoibn::Matrix& stateJacobian(){return _state_jacobian;}
  // const mwoibn::Matrix& worldJacobian(){return _world_jacobian;}

  // const mwoibn::VectorN& stateOffset(){return _state_offset;}
  SupportState cost_I;
  StateTransformation cost_II;
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

  //mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus2> _new_model_;
  //typedef mwoibn::dynamic_points::Acceleration<mwoibn::point_handling::LinearAcceleration, mwoibn::point_handling::LinearVelocity> linear_acceleration;
  //mwoibn::robot_points::Handler<linear_acceleration> _wheel_center_acceleration_;
  // mwoibn::robot_points::Handler<mwoibn::dynamic_points::TorusRoll> _torus_roll;


};
}
}
#endif
