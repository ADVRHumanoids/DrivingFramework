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

    stiffness.setZero(_dofs);
    damping.setZero(_dofs);
    _robot.getStiffness(stiffness);
    _robot.getDamping(damping);

    //     std::cout << "stiffness original" << std::endl;

    //     std::cout << stiffness << std::endl;
    //     std::cout << "damping original" << std::endl;

    //     std::cout << damping << std::endl;

    for (auto entry : config["gains"])
    {
      if (!entry.second.IsMap())
        continue;
      if (!entry.second["name"])
        continue;

      //        std::cout << "name " << entry.second["name"].as<std::string>()
      //        << std::endl;

      //        std::cout << "dof index " <<
      //        _robot.getDofIndex(entry.second["name"].as<std::string>()) <<
      //        std::endl;
      if (!entry.second["a_Kp"])
        throw std::invalid_argument(std::string(
            "Required argument a_Kp has not been defined for the joint: " +
            entry.first.as<std::string>() + ", name " +
            entry.second["name"].as<std::string>()));
      if (!entry.second["a_Kd"])
        throw std::invalid_argument(std::string(
            "Required argument a_Kd has not been defined for the joint: " +
            entry.first.as<std::string>() + ", name " +
            entry.second["name"].as<std::string>()));
      //        std::cout << "Kp " << entry.second["a_Kp"].as<double>() <<
      //        std::endl;
      //        std::cout << "Kd " << entry.second["a_Kd"].as<double>() <<
      //        std::endl;

      if (_robot.getDofIndex(entry.second["name"].as<std::string>()) == -1)
        continue;
      stiffness[_robot.getDofIndex(entry.second["name"].as<std::string>())] =
          entry.second["a_Kp"].as<double>();
      damping[_robot.getDofIndex(entry.second["name"].as<std::string>())] =
          entry.second["a_Kd"].as<double>();
    }

    _robot.setStiffness(stiffness);
    _robot.setDamping(damping);

    //   if (_position && !_velocity)
    //     _turnJoints(true, config["name"].as<std::string>());
    //   if (!_position && _velocity)
    //     _turnJoints(false, config["name"].as<std::string>());

    //   _resize(false);

    pub.setZero(_dofs);
  }

  virtual ~XBotLowerLevel() {}

  virtual bool send();

protected:
  XBot::RobotInterface& _robot;
  mwoibn::VectorN pub;
  mwoibn::VectorN stiffness;
  mwoibn::VectorN damping;

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
                << " has " << reason
                << " proportional gain, and thus it has been disabled "
                   "from the controller." << std::endl;

      map_local[i] = mwoibn::NON_EXISTING;
    }

    _map = mwoibn::robot_class::BiMap(name, map_local);
  }
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
