#ifndef ROBOT_CLASS_ROBOT_XBOT_RT_H
#define ROBOT_CLASS_ROBOT_XBOT_RT_H

#include "mwoibn/robot_class/robot_xbot_feedback.h"
#include "mwoibn/communication_modules/xbot_feedback_shared.h"
#include "mwoibn/communication_modules/xbot_feedback_from_nrt.h"
#include "mwoibn/communication_modules/xbot_controller_shared.h"
#include "mwoibn/communication_modules/xbot_controller_to_nrt.h"
#include "mwoibn/communication_modules/xbot_operational_euler_from_nrt.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn
{
namespace robot_class
{
class RobotXBotRT : public RobotXBotFeedback

{
public:
  RobotXBotRT(XBot::RobotInterface::Ptr robot, std::string config_file,
              std::string config_name,
              std::string secondary_file, XBot::SharedMemory::Ptr shared_memory);

  virtual ~RobotXBotRT() {}

  virtual void wait(){}

protected:

  virtual void _init(YAML::Node config, YAML::Node robot, XBot::SharedMemory::Ptr shared_memory);

  virtual void _loadFeedbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory);

  virtual void _loadControllers(YAML::Node config, XBot::SharedMemory::Ptr shared_memory);

};
} // namespace package
} // namespace library
#endif // ROBOT_XBOT_NRT_H
