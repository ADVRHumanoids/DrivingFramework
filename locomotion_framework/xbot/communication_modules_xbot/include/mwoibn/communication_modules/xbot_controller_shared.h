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
      _pub_position = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/position");
      std::cout << "\tInitialized position interface in "
                << config["sink"].as<std::string>() << "/position\n";
    }
    if (_velocity)
    {
      std::cout << "\tInitialized velocity interface in "
                << config["sink"].as<std::string>() << "/velocity\n";
      _pub_velocity = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/velocity");
    }
    if (_torque)
    {
      std::cout << "\tInitialized torque interface in "
                << config["sink"].as<std::string>() << "/torque\n";
      _pub_torque = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["sink"].as<std::string>() + "/torque");
    }
    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotControllerShared()
  {
      
    if (_position){
      _positions[check] = mwoibn::INVALID;
      _pub_position.set(_positions);
    }
    if (_velocity){
      _velocities[check] = mwoibn::INVALID;
      _pub_velocity.set(_velocities);
    }
    if (_torque){
      _torques[check] = mwoibn::INVALID;
      _pub_torque.set(_torques);
    }
  }

  virtual bool initialize()
  {
    if(_initialized) return _initialized;
    if (_position){
      _positions[check] = mwoibn::IS_VALID;
    }
    if (_velocity)
      _velocities[check] = mwoibn::IS_VALID;
    if (_torque)
      _torques[check] = mwoibn::IS_VALID;

    _initialized = true;
    return _initialized;
  }

  virtual bool send()
  {
    initialize();

    if (_position){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
            _positions);
      _pub_position.set(_positions);
    }
    if (_velocity){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
            _velocities);
      _pub_velocity.set(_velocities);
    }
    if (_torque){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _torques);
      _pub_torque.set(_torques);
    }
    return true;
  }

protected:
  mwoibn::VectorRT _positions, _velocities, _torques;

  XBot::SharedObject<mwoibn::VectorRT> _pub_position;
  XBot::SharedObject<mwoibn::VectorRT> _pub_velocity;
  XBot::SharedObject<mwoibn::VectorRT> _pub_torque;
  const int check = RT_SIZE - 1;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
