#ifndef ROBOT_CLASS_ROBOT_XBOT_RT_H
#define ROBOT_CLASS_ROBOT_XBOT_RT_H

#include "mwoibn/robot_class/robot_xbot_feedback.h"
#include "mwoibn/communication_modules/xbot_feedback_shared.h"
#include "mwoibn/communication_modules/xbot_feedback_from_nrt.h"
#include "mwoibn/communication_modules/xbot_controller_shared.h"
#include "mwoibn/communication_modules/xbot_controller_to_nrt.h"
#include "mwoibn/communication_modules/xbot_operational_euler_from_nrt.h"
#include "XBotCore-interfaces/XBotSharedMemory.h"

namespace mwoibn::robot_class
{
class RobotXBotRT : public RobotXBotFeedback

{
public:
  RobotXBotRT(XBot::RobotInterface::Ptr robot, std::string config_file,
              std::string config_name, std::string controller_source,
              std::string secondary_file, XBot::SharedMemory::Ptr shared_memory);

  RobotXBotRT(XBot::RobotInterface::Ptr robot, YAML::Node full_config,
              std::string config_name, std::string controller_source,
              XBot::SharedMemory::Ptr shared_memory);


  RobotXBotRT(XBot::RobotInterface::Ptr robot, std::string config_file,
              std::string config_name, mwoibn::communication_modules::Shared& shared,
              std::string controller_source,
              std::string secondary_file, XBot::SharedMemory::Ptr shared_memory);

  RobotXBotRT(XBot::RobotInterface::Ptr robot, YAML::Node full_config,
              std::string config_name, mwoibn::communication_modules::Shared& shared,
              std::string controller_source, XBot::SharedMemory::Ptr shared_memory);

  virtual ~RobotXBotRT() {}

  virtual void loadControllers(std::string config_file, std::string config_name,
                       mwoibn::communication_modules::Shared& shared,
                       std::string controller_source, std::string secondary_file, XBot::SharedMemory::Ptr shared_memory);


  virtual void loadControllers(YAML::Node full_config, std::string config_name,
                       mwoibn::communication_modules::Shared& shared,  std::string controller_source, XBot::SharedMemory::Ptr shared_memory);


//  virtual void wait(bool spin = true){}
  virtual void wait(bool spin = true){ _sense = spin; kinematics_update.set(true);}


protected:

  virtual void _init(YAML::Node config, YAML::Node robot, XBot::SharedMemory::Ptr shared_memory);
  virtual void _init(YAML::Node config, YAML::Node robot, mwoibn::communication_modules::Shared& shared, XBot::SharedMemory::Ptr shared_memory);

  virtual void _loadFeedbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory);

  virtual void _loadControllers(YAML::Node config, XBot::SharedMemory::Ptr shared_memory);
  virtual std::unique_ptr<mwoibn::communication_modules::CommunicationBase> _generateContactCallback(mwoibn::robot_points::Contact& contact, YAML::Node config, XBot::SharedMemory::Ptr shared);
  using Robot::_generateContactCallback;

  virtual void _initContactsCallbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory);
  virtual void _initContactsCallbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory, mwoibn::communication_modules::Shared& shared);
};
} // namespace package
#endif // ROBOT_XBOT_NRT_H
