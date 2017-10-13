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
    std::cout << "Loading direct controller to the robot - " << config["name"]
              << std::endl;

    if (_position)
      std::cout << "\tInitialized position interface\n";
    if (_velocity)
      std::cout << "\tInitialized velocity interface\n";
    if (_torque)
      std::cout << "\tInitialized torque interface\n";

    if (_position && !_velocity)
      _turnJoints(true, config["name"].as<std::string>());
    if (!_position && _velocity)
      _turnJoints(false, config["name"].as<std::string>());

    _resize(false);

    pub.setZero(_dofs);

    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotLowerLevel() {}

  virtual bool send();

protected:
  XBot::RobotInterface& _robot;
  mwoibn::VectorN pub;

  void _turnJoints(bool position, std::string name)
  {

    mwoibn::VectorN gain(_dofs);
//    int map_dofs = _map.getDofs();

    mwoibn::VectorInt map_local = _map.get();

    _robot.getStiffness(gain);

    for (int i = 0; i < _map.getDofs(); i++)
    {
//      map_local[i] = _map.get()[i];

      if (map_local[i] == mwoibn::NON_EXISTING)
        continue;

      if (position && gain[map_local[i]])
        continue;
      if (!position && !gain[map_local[i]])
        continue;


      std::string reason = (position) ? "zero" : "non zero";
      std::cout << name << ": Joint "
                << _robot.getJointByDofIndex(map_local[i])->getJointName()
                << " has " << reason << " proportional gain, and thus it has been disabled "
                   "from the controller." << std::endl;

      map_local[i] = mwoibn::NON_EXISTING;

    }

    _map = mwoibn::robot_class::BiMap(name, map_local);

  }
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
