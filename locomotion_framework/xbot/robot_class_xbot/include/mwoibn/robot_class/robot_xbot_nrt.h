#ifndef ROBOT_CLASS_ROBOT_XBOT_NRT_H
#define ROBOT_CLASS_ROBOT_XBOT_NRT_H

#include "mwoibn/robot_class/robot_xbot_feedback.h"
#include "mwoibn/communication_modules/xbot_controller_to_rt.h"
#include "mwoibn/communication_modules/xbot_feedback_from_rt.h"
#include "mwoibn/robot_class/robot_ros_nrt.h"

namespace mwoibn
{
namespace robot_class
{
class RobotXBotNRT : public RobotXBotFeedback

{
public:
  RobotXBotNRT(std::string config_file, std::string config_name, std::string controller_source,
               std::string secondary_file = "");

  RobotXBotNRT(YAML::Node full_config, std::string config_name, std::string controller_source);

  virtual ~RobotXBotNRT() {}
//  virtual void update()
//  {
//    send();
//    RobotXBotFeedback::update();
//  }

//  virtual bool send()
//  {
//    robot_class::RobotXBotFeedback::send(); // this is spining on its own?
//    if (_spin)
//      ros::spinOnce();
//  }

  virtual double rate() {
      return _rate_ptr->expectedCycleTime().toSec();
    }

  virtual void wait(bool spin = true) {_rate_ptr->sleep(); }
  virtual bool isRunning() { return _robot->isRunning(); }
protected:
  virtual void _loadFeedbacks(YAML::Node config);
  virtual void _init(YAML::Node config, YAML::Node robot);
  virtual void _loadControllers(YAML::Node config);
  bool _spin = false;
  std::unique_ptr<ros::Rate> _rate_ptr;
};
} // namespace package
} // namespace library
#endif // ROBOT_XBOT_NRT_H
