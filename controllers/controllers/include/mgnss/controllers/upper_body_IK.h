#ifndef __MGNSS_CONTROLLERS_UPPER_BODY_IK_H
#define __MGNSS_CONTROLLERS_UPPER_BODY_IK_H

#include "mgnss/controllers/ik_base.h"
#include "mwoibn/robot_points/norm.h"
//#include <mwoibn/hierarchical_control/tasks/constraints_task.h>

#include <mwoibn/hierarchical_control/tasks/cartesian_world_task.h>
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
  for(int i = 0; i < _arms_ptr->points().size(); i++){
    _direction = (_desried_pos.segment<3>(3*i) - _arms_ptr->getReference(i));

    if(std::isnan(_direction.norm()) || _direction.norm() < _step)
      _arms_ptr->setReference(i, _desried_pos.segment<3>(3*i));
    else
      _arms_ptr->setReference(i, _arms_ptr->getReference(i)+_direction.normalized()*_step);
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



void setReference(int i, mwoibn::Vector3& reference){
  _desried_pos.segment<3>(3*i) += reference;
}


protected:

//std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints> _constraints_ptr;

std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianWorld> _arms_ptr;
//std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective> _pelvis_orientation_ptr;

virtual void _setInitialConditions();
virtual void _allocate(){
  _desried_pos.setZero(3*_arms_ptr->points().size());
}
virtual void _createTasks(YAML::Node config);
mwoibn::VectorN _desried_pos;
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
