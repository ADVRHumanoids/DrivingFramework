#ifndef __MGNSS__CONTROLLERS__WHEELED_MOTION_EVENT_3_H
#define __MGNSS__CONTROLLERS__WHEELED_MOTION_EVENT_3_H

#include "mgnss/controllers/wheels_controller_extend.h"

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

//#include <mgnss/higher_level/state_machine_II.h>
//#include <mgnss/higher_level/state_machine.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionEvent3 : public WheelsControllerExtend
{

public:
  WheeledMotionEvent3( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot)
  {
          YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
          config["name"] = name;
          _create(config);
  }



  WheeledMotionEvent3( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot)
  {
          _create(config);
  }

virtual ~WheeledMotionEvent3() {
}

virtual void log(mwoibn::common::Logger& logger, double time);

void updateBase(){

        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);

        WheelsController::updateBase();
//        state_machine__->update();
}

const mwoibn::VectorN& getComFull(){
        return _robot.centerOfMass().get();
}
const mwoibn::VectorN& errorCom(){
        return _com_ptr->getError();
}

const mwoibn::VectorN& refCom(){
        return _com_ptr->getReference();
}
double refComX(){
        return _com_ptr->getReference() (0,0);
}
double refComY(){
        return _com_ptr->getReference() (0,1);
}
virtual double getBaseGroundX()
{
        return _robot.centerOfMass().get()[0];
}
virtual double getBaseGroundY()
{
        return _robot.centerOfMass().get()[1];
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

const mwoibn::Vector3& getBaseReference(){
        return _pelvis_position_ptr->getReference(0);
}


protected:
  WheeledMotionEvent3(mwoibn::robot_class::Robot& robot) : WheelsControllerExtend(robot){ }

  std::unique_ptr<mwoibn::robot_points::LinearPoint> _pelvis;

  void _allocate(YAML::Node config);

  std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;
  //std::unique_ptr<mgnss::higher_level::StateMachineII> state_machine__;

  mwoibn::VectorN _com_ref;

  virtual void _setInitialConditions();
  virtual void _allocate();
  virtual void _createTasks(YAML::Node config);
  virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
