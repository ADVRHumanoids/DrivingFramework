#ifndef COMMUNICATION_MODULES_XBOT_FEEDBACK_ONLINE_H
#define COMMUNICATION_MODULES_XBOT_FEEDBACK_ONLINE_H

#include "mwoibn/robot_class/state.h"
#include "mwoibn/communication_modules/basic_feedback.h"
#include <XBotInterface/RobotInterface.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotFeedbackOnline : public BasicFeedback
{
public:
  XBotFeedbackOnline(mwoibn::robot_class::State& command,
                     mwoibn::robot_class::State& lower_limits,
                     mwoibn::robot_class::State& upper_limits,
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot)
      : BasicFeedback(command, map, config), _robot(robot), _lower_limits(lower_limits), _upper_limits(upper_limits)
  {
    std::cout << "Loaded direct feedback from the robot - " << config["name"] << std::endl;
    if(_position)
      std::cout << "\tInitialized position interface\n";
    if(_velocity)
      std::cout << "\tInitialized velocity interface\n";
    if(_torque)
      std::cout << "\tInitialized torque interface\n";

    _pub.setZero(_robot.getJointNum());
    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotFeedbackOnline(){}

  bool initialize(){
    _initialized = true;

    if(_position)
      _initialized = _command.get(mwoibn::robot_class::INTERFACE::POSITION).norm() > 1e-12 && _initialized;
    if(_velocity)
      _initialized = _command.get(mwoibn::robot_class::INTERFACE::VELOCITY).norm() > 1e-12 && _initialized;

    if(_torque)
      _initialized = _command.get(mwoibn::robot_class::INTERFACE::TORQUE).norm() > 1e-12 && _initialized;

    return _initialized;
  }

  virtual bool get();

  virtual bool reset();

protected:
  mwoibn::VectorN _pub;
  XBot::RobotInterface& _robot;
  const mwoibn::robot_class::State& _lower_limits, _upper_limits;
  virtual bool _inLimits(mwoibn::robot_class::INTERFACE interface);
  virtual bool _inLimits(int i, mwoibn::robot_class::INTERFACE interface);

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
