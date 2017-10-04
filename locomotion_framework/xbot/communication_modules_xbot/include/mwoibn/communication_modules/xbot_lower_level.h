#ifndef COMMUNICATION_MODULES_XBOT_LOWER_LEVEL_H
#define COMMUNICATION_MODULES_XBOT_LOWER_LEVEL_H

#include "mwoibn/robot_class/state.h"
#include "mwoibn/communication_modules/basic_controller.h"
#include <XBotInterface/RobotInterface.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotLowerLevel : public BasicController
{
public:
  XBotLowerLevel(mwoibn::robot_class::State& command,
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot)
      : BasicController(command, map, config), _robot(robot)
  {
    std::cout << "Loading direct controller to the robot - " << config["name"] << std::endl;
    if(_position)
      std::cout << "\tInitialized position interface\n";
    if(_velocity)
      std::cout << "\tInitialized velocity interface\n";
    if(_torque)
      std::cout << "\tInitialized torque interface\n";

    pub.setZero(_dofs);
    std::cout << "\tSuccess" << std::endl;


  }

  virtual ~XBotLowerLevel(){}

  virtual bool send();

protected:
  XBot::RobotInterface& _robot;
  mwoibn::VectorN pub;


};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
