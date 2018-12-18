#ifndef __MGNSS_WHEELED_MOTION_WORLD_H
#define __MGNSS_WHEELED_MOTION_WORLD_H

#include "mgnss/controllers/wheels_controller_extend.h"

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include "mwoibn/robot_points/constant.h"

namespace mgnss
{
namespace controllers
{

class WheeledMotionWorld : public WheelsControllerExtend
{

public:
  WheeledMotionWorld( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot), _world(3, robot.getDofs())
  {
          YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
          config["name"] = name;
          _create(config);
  }

  WheeledMotionWorld( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot), _world(3, robot.getDofs())
  {
          _create(config);
  }

virtual ~WheeledMotionWorld() {
}

virtual void log(mwoibn::common::Logger& logger, double time);

void updateBase()
{
        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);

        WheelsController::updateBase();
}


virtual double getBaseGroundX() {
        return _robot.centerOfMass().get()[0];
}
virtual double getBaseGroundY() {
        return _robot.centerOfMass().get()[1];
}
virtual double getBaseGroundZ()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[2];
}

const mwoibn::VectorN& getComFull() {
        return _robot.centerOfMass().get();
}
const mwoibn::VectorN& errorCom() {
        return _com_ptr->getError();
}

const mwoibn::VectorN& refCom() {
        return _com_ptr->getReference();
}
double refComX() {
        return _com_ptr->getReference() (0, 0);
}
double refComY() {
        return _com_ptr->getReference() (0, 1);
}

const mwoibn::VectorBool& isResteer() {
        return _resteer;
}

const mwoibn::VectorN& getBaseError()
{
        return _pelvis_position_ptr->getError();
}

protected:
std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;

mwoibn::VectorN _com_ref;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);

mwoibn::robot_points::Constant _world;

};
}
}

#endif // WHEELED_MOTION_H
