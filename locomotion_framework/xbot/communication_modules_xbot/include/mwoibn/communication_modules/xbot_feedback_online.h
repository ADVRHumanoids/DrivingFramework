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
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot)
      : BasicFeedback(command, map, config), _robot(robot)
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

  virtual bool initialized(){return true;} // not implemented yet

  virtual bool get();

protected:
  mwoibn::VectorN _pub;
  XBot::RobotInterface& _robot;


};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
