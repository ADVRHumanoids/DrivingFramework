#ifndef __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H
#define __MGNSS__HIGHER_LEVEL__STATE_MACHINE_H

#include "mgnss/higher_level/support_shaping_v3_0.h"
#include "mgnss/higher_level/qr_tracking.h"

#include "mwoibn/robot_class/robot.h"
// #include "mwoibn/hierarchical_control/tasks/controller_task.h"

#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/handler.h"

#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/robot_points/linear_point.h"
#include "mwoibn/robot_points/minus.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/robot_points/ground_wheel.h"


namespace mgnss
{

namespace higher_level
{

struct Limit{

  mwoibn::VectorN limit;
  mwoibn::VectorN state;
  mwoibn::Matrix jacobian;
  mwoibn::VectorN error;
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
  void update(){
    _update();
    for(int i = 0; i < _size; i++)
       _computeMargin(i);

    _marginJacobians();
    _computeWorkspace();
    _workspaceJacobian();

    _margins.error = (_margins.state - _margins.limit)/_robot.rate();
    _workspace.error = (_workspace.limit.cwiseProduct(_workspace.limit) - _workspace.state)/_robot.rate();

    if(_margins.error.minCoeff() < 0 || _workspace.error.minCoeff() < 0) {
      _time = 0;
      _counter = 0;
    }
    else {
      _time += _robot.rate();
      _counter++;
      }

      _state = (_counter < 25) ? false : true;
      _restart = (_counter == 25) ? true : false;

    std::cout << "counter\t" << _counter << "\t" << _restart << std::endl;
    std::cout << "_margins\t" << _margins.state.transpose() << std::endl;
    std::cout << "_workspace\t" << _workspace.state.transpose() << std::endl;
  }

  bool state(){ return _state; }

  double time(){return _time;}

  bool restart() { return _restart;}

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steeringFrames(){return _wheel_transforms;}
  //
  // const mwoibn::VectorN& margins(){return _margins;}
  // const mwoibn::VectorN& workspace(){return _workspace;}
  //
  // const mwoibn::Matrix& marginsJacobian(){return _margins_jacobian;}
  // const mwoibn::Matrix& workspaceJacobian(){return _workspace_jacobian;}
  const Limit& margin(){return _margins;}
  const Limit& workspace(){return _workspace;}
  // bool valid();

  // const mwoibn::VectorN& marginsLimits(){return _safety_margins;}
  // const mwoibn::VectorN& workspaceLimits(){return _max_workspace;}
  //
  // const mwoibn::VectorN& marginSafety();
  // const mwoibn::VectorN& workspaceSafety();

protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::robot_points::Point& _base;

  unsigned int _size, _counter;
  // mgnss::higher_level::SupportShapingV3& _shape;
  // mgnss::higher_level::QrTracking& _restore;

  bool _state, _restart;
  double _time = 0;

  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _contact_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _base_points;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _hips, _wheels;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _workspace_points;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>> _wheel_transforms;

  std::vector<std::pair<int,int>> _margin_pairs;

  Limit _margins, _workspace;
  // mwoibn::VectorN _safety_margins, _max_workspace, _margins, _workspace, _norms;
  mwoibn::VectorN _norms;
  // mwoibn::Matrix _margins_jacobian, _workspace_jacobian;


  void _computeMargin(int i);
  void _marginJacobians();

  void _computeWorkspace();
  void _workspaceJacobian();

  void _update();




};
}
}
#endif
