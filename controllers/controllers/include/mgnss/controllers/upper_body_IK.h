#ifndef __MGNSS_CONTROLLERS_UPPER_BODY_IK_H
#define __MGNSS_CONTROLLERS_UPPER_BODY_IK_H

#include "mgnss/controllers/ik_base.h"
#include "mwoibn/robot_points/norm.h"
//#include <mwoibn/hierarchical_control/tasks/constraints_task.h>

#include <mwoibn/hierarchical_control/tasks/cartesian_world_task.h>
#include <mwoibn/hierarchical_control/tasks/handler_task.h>

//#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

namespace mgnss
{

namespace controllers {

class UpperBodyIK : public mgnss::controllers::IKBase
{

public:
UpperBodyIK( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : IKBase(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;
        _create(config);
}



UpperBodyIK( mwoibn::robot_class::Robot& robot, YAML::Node config) : IKBase(robot)
{
        _create(config);
}


virtual ~UpperBodyIK() {
}


virtual void step(){
  // for(int i = 0; i < _arms_ptr->points().size(); i++){
  //   _direction = (_desired.segment<3>(3*i) - _arms_ptr->getReference(i));
  //
  //   if(std::isnan(_direction.norm()) || _direction.norm() < _step)
  //     _arms_ptr->setReference(i, _desired.segment<3>(3*i));
  //   else
  //     _arms_ptr->setReference(i, _arms_ptr->getReference(i)+_direction.normalized()*_step);
  // }
  
  for(auto&& [desired_, reference_]:  ranges::view::zip(_desired, _handler_task.reference)){
    _direction = (desired_ - reference_);

     if(std::isnan(_direction.norm()) || _direction.norm() < _step)
       reference_ = desired_;
     else
       reference_ += _direction.normalized()*_step;
   }

  _body_points.update(true);
  _workspace_points.update(true);
  _norms_points.update(true);
  // std::cout << "state\t" << _qr_wrappers["ARMS"]->hard_inequality.getState().transpose() << std::endl;
  // std::cout << "norms\t" << _norms_points.getState().transpose() << std::endl;
  if(std::isinf(_qr_wrappers["ARMS"]->optimalCost())){
    std::cerr << "upper_body:inf\t" << _infs << std::endl;
    ++_infs;
  }
  // std::cout << _qr_wrappers["ARMS"]->hard_inequality[0].getState().transpose() << std::endl;

  // std::cout << _arms_ptr->getError().transpose() << std::endl;
}

void setReference(int i, const mwoibn::Vector3& reference){
  _desired[i] += reference;
}

void setReference(int i, const mwoibn::VectorN& reference){
  _desired[i] += reference;
}


protected:

//std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints> _constraints_ptr;

// std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianWorld> _arms_ptr;
mwoibn::hierarchical_control::tasks::HandlerTask<mwoibn::robot_points::Point> _handler_task;

//std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective> _pelvis_orientation_ptr;

virtual void _setInitialConditions();
virtual void _allocate(){
  _handler_task.init();
  _desired.clear();

  for(auto& point: _handler_task.handler)
    _desired.push_back(point->get());
}

virtual void _createTasks(YAML::Node config);
std::vector<mwoibn::VectorN> _desired;
double _step = 0.01, _current_step;
mwoibn::Vector3 _direction;
mwoibn::VectorN _arm_workspace;
mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _workspace_points;
mwoibn::robot_points::Handler<mwoibn::robot_points::Norm> _norms_points;
mwoibn::robot_points::Handler<mwoibn::robot_points::Point> _body_points;
virtual void _addConstraints(YAML::Node config, mgnss::higher_level::QrTask& task);
int _infs = 0;
// virtual void _addConstraints(YAML::Node config, mgnss::higher_level::QrTask& task, const std::string& name){
//   // for(auto& action: _actions){
//   //     if(action.first == name) continue;
//   //     if(!_tasks[action.first]) continue;
//   //     task.equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
//   // }
//   //
//   // task.hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));
//
// }

};
}
}
#endif // WHEELED_MOTION_H
