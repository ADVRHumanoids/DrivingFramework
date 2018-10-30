#ifndef COMMUNICATION_MODULES_XBOT_LOWER_LEVEL_H
#define COMMUNICATION_MODULES_XBOT_LOWER_LEVEL_H

#include "mwoibn/common/all.h"
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
                 mwoibn::robot_class::State& lower_limits,
                 mwoibn::robot_class::State& upper_limits,
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot)
      : BasicController(command, map, config), _lower_limits(lower_limits),
        _upper_limits(upper_limits), _robot(robot)
  {
    std::cout << "Loading direct controller to the robot - " << config["name"]
              << std::endl;

    if (_position)
      std::cout << "\tInitialized position interface\n";
    if (_velocity)
      std::cout << "\tInitialized velocity interface\n";
    if (_torque)
      std::cout << "\tInitialized torque interface\n";

    stiffness.setZero(_robot.getJointNum());
    damping.setZero(_robot.getJointNum());
    _robot.getStiffness(stiffness);
    _robot.getDamping(damping);

    for (auto entry : config["gains"])
    {
      if (!entry.second.IsMap())
        continue;
      if (!entry.second["name"])
        continue;

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

      if (_robot.getDofIndex(entry.second["name"].as<std::string>()) == -1)
        continue;

      stiffness[_robot.getDofIndex(entry.second["name"].as<std::string>())] =
          entry.second["a_Kp"].as<double>();
      damping[_robot.getDofIndex(entry.second["name"].as<std::string>())] =
          entry.second["a_Kd"].as<double>();
    }

    _robot.setStiffness(stiffness);
    _robot.setDamping(damping);

//    std::cout << stiffness << std::endl;


    pub.setZero(_robot.getJointNum());
  }

  virtual ~XBotLowerLevel() {}

  virtual bool send();

protected:
  XBot::RobotInterface& _robot;
  mwoibn::VectorN pub;
  mwoibn::VectorN stiffness;
  mwoibn::VectorN damping;
  const mwoibn::robot_class::State& _lower_limits;
  const mwoibn::robot_class::State& _upper_limits;

  void _limit(mwoibn::robot_class::INTERFACE interface)
  {

    for (int i = 0; i < _command.interface(interface).size(); i++)
    {
      if(_map.get()[i] == mwoibn::NON_EXISTING) continue;
      if (_lower_limits.interface(interface).get(i) == mwoibn::NON_EXISTING)
        continue;
      if (_command.interface(interface).get(i) < _lower_limits.interface(interface).get(i)){
          _command.interface(interface).set(_lower_limits.interface(interface).get(i), i);
      }
      else if (_command.interface(interface).get(i) > _upper_limits.interface(interface).get(i)){
               _command.interface(interface).set(_upper_limits.interface(interface).get(i), i);
      }
    }
  }
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
