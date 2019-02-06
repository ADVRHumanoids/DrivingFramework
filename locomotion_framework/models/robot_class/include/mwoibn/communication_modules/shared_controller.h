#ifndef __MWOIBN__COMMUNICATION_MODULES__SHARED_CONTROLLER_H
#define __MWOIBN__COMMUNICATION_MODULES__SHARED_CONTROLLER_H

#include "mwoibn/communication_modules/basic_controller.h"
#include "mwoibn/communication_modules/shared.h"

namespace mwoibn
{
namespace communication_modules
{

class SharedController : public BasicController
{

public:
  SharedController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap& map, std::string name, mwoibn::communication_modules::Shared& shared, bool position = true, bool velocity = true, bool torque = true)
      : BasicController(command, map, position, velocity, torque), _shared(shared)
  {
    _init(name);
  }

  // for now only full robot is supported for this controller
  SharedController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap& map, YAML::Node config, mwoibn::communication_modules::Shared& shared)
      : BasicController(command, map, config), _shared(shared)
  {
    if(!config["name"]) throw(std::invalid_argument("Missing required parameter: name"));

    _init(config["name"].as<std::string>());
  }

  SharedController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap&& map, std::string name, mwoibn::communication_modules::Shared& shared, bool position = true, bool velocity = true, bool torque = true)
      : BasicController(command, map, position, velocity, torque), _shared(shared)
  {
    _init(name);
  }

  // for now only full robot is supported for this controller
  SharedController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap&& map, YAML::Node config, mwoibn::communication_modules::Shared& shared)
      : BasicController(command, map, config), _shared(shared)
  {
    if(!config["name"]) throw(std::invalid_argument("Missing required parameter: name"));

    _init(config["name"].as<std::string>());
  }

  SharedController(SharedController& other)
      : BasicController(other), _filtered(other._filtered), _shared(other._shared), _interface(other._interface), _name(other._name),
       _name_position(other._name_position), _name_velocity(other._name_velocity), _name_torque(other._name_torque)
  {  }

  SharedController(SharedController&& other)
      : BasicController(other), _filtered(other._filtered), _shared(other._shared), _interface(other._interface), _name(other._name),
       _name_position(other._name_position), _name_velocity(other._name_velocity), _name_torque(other._name_torque)
  {  }

  SharedController(BasicController& other, mwoibn::communication_modules::Shared& shared, std::string name)
      : BasicController(other), _shared(shared)
  {
    _init(name);
  }

  virtual ~SharedController() {}

  virtual bool run();
  virtual bool initialize(){ _initialized = true; return _initialized;}

protected:
  mwoibn::VectorN _filtered;
  mwoibn::communication_modules::Shared& _shared;
  mwoibn::Interface _interface;
  std::string _name, _name_position, _name_velocity, _name_torque;
  virtual void _init(std::string name);
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
