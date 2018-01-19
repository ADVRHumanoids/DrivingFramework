#ifndef ROBOT_CLASS_ROBOT_XBOT_FEEDBACK_H
#define ROBOT_CLASS_ROBOT_XBOT_FEEDBACK_H

#include "mwoibn/robot_class/robot_xbot.h"
#include <XBotInterface/RobotInterface.h>
#include "mwoibn/communication_modules/xbot_lower_level.h"
#include "mwoibn/communication_modules/xbot_feedback_online.h"
#include "mwoibn/communication_modules/xbot_operational_euler.h"
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>

namespace mwoibn
{
namespace robot_class
{
class RobotXBotFeedback : public RobotXBot

{
public:
  RobotXBotFeedback(std::string config_file, std::string config_name,
                    std::string secondary_file = "");

  virtual ~RobotXBotFeedback() {}
  virtual void wait(){}

//  virtual void update()
//  {
//    send();
//    get();

//    RobotXBot::update();
//    wait();
//    // is there a wait option?
//  }

  virtual bool get()
  {
    if (_sense)
    {
      _robot->sense(false);
    }

    return Robot::get();
  }

  virtual bool send()
  {
    if (Robot::send() && _move)
    {
      _robot->move(); // shouldn't this go with controllers.send()?
       return true;
    }
    return false;
  }

  virtual bool isRunning() { _robot->isRunning(); }

protected:
  RobotXBotFeedback() {}
  XBot::RobotInterface::Ptr _robot;
  virtual void _init(YAML::Node config, YAML::Node robot);

  virtual void _loadFeedbacks(YAML::Node config);
  //  virtual void _initFeedbackOnline(YAML::Node config);
  //  virtual void _updateFeedbackOnline();

  virtual void _loadControllers(YAML::Node config);

  void _initStates();
  //  virtual void _initControllersLowerLevel(YAML::Node config);

  //  virtual void _initFloatingBase(YAML::Node config);
  //  virtual void _updateFloatingBase();

  bool _sense = false;
  bool _move = false;

  //  int _xbot_map;
};
} // namespace package
} // namespace library
#endif // ROBOT_XBOT_NRT_H
