#ifndef COMMUNICATION_MODULES_XBOT_FEEDBACK_FROM_RT_H
#define COMMUNICATION_MODULES_XBOT_FEEDBACK_FROM_RT_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include <XBotCore-interfaces/XDomainCommunication.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotFeedbackFromRT : public BasicFeedback
{
public:
  XBotFeedbackFromRT(mwoibn::robot_class::State& command,
                     mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicFeedback(command, map, config)
  {
    std::cout << "Loading feedback from real-time - "
              << config["name"].as<std::string>() << "\n";

    if (!config["source"])
      throw(
          std::invalid_argument("Please define name of real-time feedback.\n"));

    if (_position)
    {
      _sub_position.init(config["source"].as<std::string>() + "_position");
      std::cout << "\tInitialized position interface in "
                << config["source"].as<std::string>() << "_position\n";
    }
    if (_velocity)
    {
      _sub_velocity.init(config["source"].as<std::string>() + "_velocity");
      std::cout << "\tInitialized velocity interface in "
                << config["source"].as<std::string>() << "_velocity\n";
    }
    if (_torque)
    {
      _sub_torque.init(config["source"].as<std::string>() + "_torque");
      std::cout << "\tInitialized torque interface in "
                << config["source"].as<std::string>() << "_torque\n";
    }

    std::cout << "\tSuccess" << std::endl;
    _data.setZero();
  }

  virtual ~XBotFeedbackFromRT() {}

  virtual bool get()
  {

    _initialized = true;

    if (_position)
    {
      _data[check] = mwoibn::INVALID;
      _sub_position.read(_data);
      if (_data[check] != mwoibn::IS_VALID)
      {
        _initialized = false;
        return false;
      }
      _command.set(_data, _map.reversed(), robot_class::INTERFACE::POSITION);
    }
    if (_velocity)
    {
      _data[check] = mwoibn::INVALID;
      _sub_velocity.read(_data);
      if (_data[check] != mwoibn::IS_VALID)
      {
        _initialized = false;
        return false;
      }
      _command.set(_data, _map.reversed(), robot_class::INTERFACE::VELOCITY);
    }
    if (_torque)
    {
      _data[check] = mwoibn::INVALID;
      _sub_torque.read(_data);
      if (_data[check] != mwoibn::IS_VALID)
      {
        _initialized = false;
        return false;
      }
      _command.set(_data, _map.reversed(), robot_class::INTERFACE::TORQUE);
    }

    return true;
  }

  virtual bool initialized() { return _initialized; }

protected:
  mwoibn::VectorRT _data;

  XBot::SubscriberNRT<mwoibn::VectorRT> _sub_position;
  XBot::SubscriberNRT<mwoibn::VectorRT> _sub_velocity;
  XBot::SubscriberNRT<mwoibn::VectorRT> _sub_torque;
  const int check = RT_SIZE - 1;
  //bool _initialized = false;
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
