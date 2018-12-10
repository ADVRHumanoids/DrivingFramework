#ifndef COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_H
#define COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_H

#include "mwoibn/common/all.h"
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
                 XBot::RobotInterface& robot, double rate);

  virtual bool initialize(){reset();}

  virtual bool reset();
  virtual void getPosition(mwoibn::Matrix3 orientation, mwoibn::Vector3 position);
  virtual ~XBotOperationalEuler(){}
  virtual bool run();

protected:
  XBot::ImuSensor::ConstPtr _imu;

  mwoibn::Vector3 _linear_state;
  mwoibn::Matrix3 _rotation, _rot_z, _offset_org;
  mwoibn::VectorN _base;
  double _rate;
  bool _is_static;
  mwoibn::Axis _z;

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
