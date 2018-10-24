#ifndef BASIC_CONTROLLERS_LOWER_LEVEL_CONTROLLER_H
#define BASIC_CONTROLLERS_LOWER_LEVEL_CONTROLLER_H

#include "mwoibn/basic_controllers/basic_controller.h"
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/common/all.h"

namespace mwoibn{
namespace basic_controllers
{

class LowerLevelController : public BasicController
{

public:
  LowerLevelController(mwoibn::robot_class::Robot& robot,
                       mwoibn::robot_class::INTERFACE interface)
      : BasicController(), _robot(robot), _interface(interface)
  {
  }

  virtual ~LowerLevelController(){}

  /** sends command directly to the robot */
  virtual void setCommand() { _robot.command.set(_command, _interface); }

  /** provides access to the robot */
  mwoibn::robot_class::Robot& getRobot(){return _robot;}

protected:
  mwoibn::robot_class::Robot& _robot;

  /** keeps interface that  should be used to set robot command */
  const mwoibn::robot_class::INTERFACE _interface;
};
} // namespace package
} // namespace library

#endif // LOWER_LEVEL_CONTROLLER_H
