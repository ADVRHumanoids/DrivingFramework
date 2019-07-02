#ifndef __MGNSS_CONTROLLERS_UPPER_BODY_IK_H
#define __MGNSS_CONTROLLERS_UPPER_BODY_IK_H

#include "mgnss/controllers/ik_base.h"

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
  std::cout << _arms_ptr->getError().transpose() << std::endl;
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


};
}
}
#endif // WHEELED_MOTION_H
