#ifndef COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_FROM_NRT_H
#define COMMUNICATION_MODULES_XBOT_OPERATIONAL_EULER_FROM_NRT_H

#include "mwoibn/common/all.h"
#include "mwoibn/communication_modules/basic_operational_euler.h"
#include <XBotInterface/RobotInterface.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotOperationalEulerFromNRT : public BasicOperationalEuler
{
public:
  XBotOperationalEulerFromNRT(mwoibn::robot_class::State& command,
                              mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicOperationalEuler(command, map, config)
  {
    std::cout << "Loading feedback from non-real-time - "
              << config["name"].as<std::string>() << "\n";

    if (!config["source"])
      throw(
          std::invalid_argument("Please define source of real-time feedback.\n"));

    _sub_position.init(config["source"].as<std::string>());
    std::cout << "\tInitialized position interface in "
              << config["source"].as<std::string>() << "\n";

    std::cout << "\tSuccess" << std::endl;

  }

  virtual ~XBotOperationalEulerFromNRT() {}

  virtual bool get()
  {
    _sub_position.read(_state);

    if(_state[check] != mwoibn::IS_VALID){
      _initialized = false;
      return false;}

    _orientation.x() = _state[3];
    _orientation.y() = _state[4];
    _orientation.z() = _state[5];
    _orientation.w() = _state[6];

    _linear_state = _state.head(3);

    getPosition(_orientation, _linear_state);
    _initialized = true;
  }

protected:
  XBot::SubscriberRT<mwoibn::VectorFS> _sub_position;
  mwoibn::VectorFS _state;
  bool _initialized = false;
  const int check = FS_SIZE - 1;
  mwoibn::Quaternion _orientation;
  mwoibn::Vector3 _linear_state;

};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
