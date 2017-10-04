#ifndef COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_H
#define COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_H

#include "mwoibn/robot_class/state.h"
#include "mwoibn/communication_modules/basic_operational_euler.h"
#include <XBotInterface/RobotInterface.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotOperationalEuler : public BasicOperationalEuler
{
public:
  XBotOperationalEuler(mwoibn::robot_class::State& command,
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot);

  virtual bool initialized(){return true;} // not implemented yet

  virtual ~XBotOperationalEuler(){}

  virtual bool get();

protected:
  XBot::ImuSensor::ConstPtr _imu;
  mwoibn::Vector3 _linear_state;
  mwoibn::Matrix3 _rotation;


};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
