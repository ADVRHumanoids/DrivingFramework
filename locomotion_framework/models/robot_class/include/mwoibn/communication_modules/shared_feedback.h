#ifndef __MOWIBN__COMMUNICATION_MODULES__SHARED_FEEDBACK_H
#define __MOWIBN__COMMUNICATION_MODULES__SHARED_FEEDBACK_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include "mwoibn/communication_modules/shared.h"


namespace mwoibn
{
namespace communication_modules
{
class SharedFeedback : public BasicFeedback
{

public:
  // for now only full robot is supported for this controller
  SharedFeedback(mwoibn::robot_class::State& command,
              mwoibn::robot_class::BiMap& map, YAML::Node config, mwoibn::communication_modules::Shared& shared)
      : BasicFeedback(command, map, config), _shared(shared){
        _init(config);
      }

  SharedFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap&& map, YAML::Node config, mwoibn::communication_modules::Shared& shared)
    : BasicFeedback(command, map, config), _shared(shared){
        _init(config);
      }

  SharedFeedback(SharedFeedback& other)
      : BasicFeedback(other), _shared(other._shared), _name(other._name)
  {  }

  SharedFeedback(SharedFeedback&& other)
      : BasicFeedback(other), _shared(other._shared), _name(other._name)
  {  }

  virtual ~SharedFeedback() {}
  virtual bool initialized() { return _initialized; }
  virtual bool run() {
    _initialized = true;

    if (_position)
      _command.position.set(_shared[_name+".position"], _map.reversed());
    if (_velocity)
      _command.velocity.set(_shared[_name+".velocity"], _map.reversed());
    if (_torque)
      _command.torque.set(_shared[_name+".torque"], _map.reversed());

      return true;
  }

protected:
  mwoibn::communication_modules::Shared& _shared;
  std::string _name;

  void _init(YAML::Node config)
  {
    if (!config["name"])
      throw(std::invalid_argument("Missing required parameter: name"));

      _name = config["name"].as<std::string>();

      if(!_shared.startsWith(_name))
        throw(std::invalid_argument("Shared object " + _name + std::string(" does not exists.")));
      if(_position && !_shared.has(_name+".position"))
          throw(std::invalid_argument("Position interface for shared object " + _name + std::string(" has not been initialized.")));
      if(_velocity && !_shared.has(_name+".velocity"))
              throw(std::invalid_argument("Velocity interface for shared object " + _name + std::string(" has not been initialized.")));
      if(_torque && !_shared.has(_name+".torque"))
                      throw(std::invalid_argument("Torque interface for shared object " + _name + std::string(" has not been initialized.")));


    std::cout << "Loaded Shared feedback " << _name << std::endl;

    std::string info = (_position)? "TRUE":"FALSE";
    std::cout << "\t position interface: " << info  << "\n";

    info = (_velocity)? "TRUE":"FALSE";
    std::cout << "\t " << "velocity interface: " <<   info  << "\n";

    info = (_torque)? "TRUE":"FALSE";
    std::cout << "\t " << "torque interface: " <<   info << "\n";
    std::cout << std::endl;
  }


};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
