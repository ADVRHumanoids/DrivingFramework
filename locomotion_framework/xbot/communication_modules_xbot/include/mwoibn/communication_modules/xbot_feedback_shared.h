#ifndef COMMUNICATION_MODULES_XBOT_FEEDBACK_SHARED_H
#define COMMUNICATION_MODULES_XBOT_FEEDBACK_SHARED_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn
{
namespace communication_modules
{

class XBotFeedbackShared : public BasicFeedback
{
public:
  XBotFeedbackShared(mwoibn::robot_class::State& command,
                     mwoibn::robot_class::BiMap map, YAML::Node config,
                     XBot::SharedMemory::Ptr shared_memory)
      : BasicFeedback(command, map, config)
  {

    std::cout << "Loading feedback in real-time - "
              << config["name"].as<std::string>() << "\n";

    if (!config["source"])
      throw(
          std::invalid_argument("Please define name of real-time feedback.\n"));

    if (_position)
    {
      _sub_position = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["source"].as<std::string>() + "/position");

      std::cout << "\tInitialized position interface in "
                << config["source"].as<std::string>() << "/position\n";
    }
    if (_velocity)
    {
      _sub_velocity = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["source"].as<std::string>() + "/velocity");
      std::cout << "\tInitialized velocity interface in "
                << config["source"].as<std::string>() << "/velocity\n";
    }
    if (_torque)
    {
      _sub_torque = shared_memory->getSharedObject<mwoibn::VectorRT>(
          config["source"].as<std::string>() + "/torque");
      std::cout << "\tInitialized torque interface in "
                << config["source"].as<std::string>() << "/torque\n";
    }
    

    std::cout << "\tSuccess" << std::endl;
  }

  virtual bool initialized()
  {
    _sub_position.get(_positions);
    if (_position && _positions[check] != mwoibn::IS_VALID)
      return false;
    
    _sub_velocity.get(_velocities);
    if (_velocity && _velocities[check] != mwoibn::IS_VALID)
      return false;
    
    _sub_torque.get(_torques);
    if (_torque && _torques[check] != mwoibn::IS_VALID)
      return false;

    return true;
  }

  virtual ~XBotFeedbackShared() {}

  virtual bool get()
  {
    if(!initialized()) return false;

    if (_position)
    {
       _sub_position.get(_positions);
      _command.set(_positions, _map.reversed(),
                   robot_class::INTERFACE::POSITION);
    }
    if (_velocity)
    {
      _sub_velocity.get(_velocities);
      _command.set(_velocities, _map.reversed(),
                   robot_class::INTERFACE::VELOCITY);
    }
    if (_torque)
    {
      _sub_torque.get(_torques);
      _command.set(_torques, _map.reversed(),
                   robot_class::INTERFACE::TORQUE);
    }

    return true;
  }

protected:
  mwoibn::VectorRT _positions, _velocities, _torques;
  
  XBot::SharedObject<mwoibn::VectorRT> _sub_position;
  XBot::SharedObject<mwoibn::VectorRT> _sub_velocity;
  XBot::SharedObject<mwoibn::VectorRT> _sub_torque;
  const int check = RT_SIZE - 1;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
