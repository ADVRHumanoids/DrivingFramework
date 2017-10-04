#ifndef COMMUNICATION_MODULES_XBOT_CONTROLLER_SHARED_H
#define COMMUNICATION_MODULES_XBOT_CONTROLLER_SHARED_H

#include "mwoibn/communication_modules/basic_controller.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn
{
namespace communication_modules
{

class XBotControllerShared : public BasicController
{
public:
  XBotControllerShared(mwoibn::robot_class::State& command,
                       mwoibn::robot_class::BiMap map, YAML::Node config,
                       XBot::SharedMemory::Ptr shared_memory)
      : BasicController(command, map, config)
  {

    std::cout << "Loading controller in real-time - "
              << config["name"].as<std::string>() << "\n";

    if (!config["sink"])
      throw(std::invalid_argument(
          "Please define sink for the real-time controller " +
          config["name"].as<std::string>() + ".\n"));

    if (_position)
    {
      _pub_position = shared_memory->advertise<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/position");
      std::cout << "\tInitialized position interface in "
                << config["sink"].as<std::string>() << "/position\n";
    }
    if (_velocity)
    {
      std::cout << "\tInitialized velocity interface in "
                << config["sink"].as<std::string>() << "/velocity\n";
      _pub_velocity = shared_memory->advertise<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/velocity");
    }
    if (_torque)
    {
      std::cout << "\tInitialized torque interface in "
                << config["sink"].as<std::string>() << "/torque\n";
      _pub_torque = shared_memory->advertise<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/torque");
    }
    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotControllerShared()
  {
    if (_position)
      (*_pub_position)[check] = mwoibn::INVALID;
    if (_velocity)
      (*_pub_velocity)[check] = mwoibn::INVALID;
    if (_torque)
      (*_pub_torque)[check] = mwoibn::INVALID;
  }

  virtual void initialize()
  {
    if (_position)
      (*_pub_position)[check] = mwoibn::IS_VALID;
    if (_velocity)
      (*_pub_velocity)[check] = mwoibn::IS_VALID;
    if (_torque)
      (*_pub_torque)[check] = mwoibn::IS_VALID;
  }

  virtual bool send()
  {
    initialize();

    if (_position)
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
            *_pub_position);
    if (_velocity)
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
            *_pub_velocity);
    if (_torque)
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), *_pub_torque);

    return true;
  }

protected:
  XBot::SharedObject<mwoibn::VectorRT> _pub_position;
  XBot::SharedObject<mwoibn::VectorRT> _pub_velocity;
  XBot::SharedObject<mwoibn::VectorRT> _pub_torque;
  const int check = RT_SIZE - 1;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
