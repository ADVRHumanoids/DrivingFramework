#ifndef __MGNSS_CONTROLLERS_WHEELS_REACTIF_H
#define __MGNSS_CONTROLLERS_WHEELS_REACTIF_H

#include "mgnss/controllers/wheels_controller_extend.h"

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

namespace mgnss
{
namespace controllers
{

class WheelsReactif : public WheelsControllerExtend
{

public:
  WheelsReactif( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot)
  {
          YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
          config["name"] = name;
          _create(config);
  }

  WheelsReactif( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot)
  {
          _create(config);
  }


virtual ~WheelsReactif() {
}

virtual void initLog(mwoibn::common::Logger& logger);
virtual void log(mwoibn::common::Logger& logger, double time);

void updateBase(){

        _pelvis_position_ptr->setReference(0, _position);

        WheelsController::updateBase();
}

void compute();

virtual double getBaseGroundX()
{
        return _robot.state.position.get()[0];
}
virtual double getBaseGroundY()
{
        return _robot.state.position.get()[1];
}
virtual double getBaseGroundZ()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[2];
}

const mwoibn::VectorBool& isResteer(){
        return _resteer;
}

const mwoibn::VectorN& getBaseError(){
        return _pelvis_position_ptr->getError();
}

protected:
void _allocate(YAML::Node config);

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
