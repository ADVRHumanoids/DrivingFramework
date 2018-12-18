#ifndef __MWOIBN__COMMON__INTERFACES_H
#define __MWOIBN__COMMON__INTERFACES_H


//#include <iostream>
#include <climits>
#include <map>
#include "mwoibn/common/types.h"

namespace mwoibn
{

namespace robot_class
{
enum class CONTROL_LEVEL{
  NONE = 0,
  LOWER_LEVEL,
  HIGHER_LEVEL
};

// enum class INTERFACE
// {
//   POSITION,
//   VELOCITY,
//   ACCELERATION,
//   TORQUE,
//   OVERALL_FORCE
// };
enum class ACTUATOR_TYPE
{
  UNACTUATED = 0,
  RIGID,
  ELASTIC
};
enum class MAPPING
{
  RBDL,
  PYTHON,
  CUSTOM
};

enum class CONTACT_TYPE
{
  UNKNOWN = 0,
  POINT_FOOT,
  WHEEL_LOCKED,
  WHEEL,
};


// eventually I can think about changing CONTACT_TYPE to a string enum
const std::map<std::string, CONTACT_TYPE> contact_type = {{"point_foot", CONTACT_TYPE::POINT_FOOT}, {"unknown", CONTACT_TYPE::UNKNOWN}, {"wheel_locked", CONTACT_TYPE::WHEEL_LOCKED}, {"wheel", CONTACT_TYPE::WHEEL}};
const std::map<std::string, ACTUATOR_TYPE> actuator_type = {{"unactuated", ACTUATOR_TYPE::UNACTUATED}, {"rigid", ACTUATOR_TYPE::RIGID}, {"elastic", ACTUATOR_TYPE::ELASTIC}};
const std::map<std::string, MAPPING> mappings = {{"PYTHON", MAPPING::PYTHON}, {"RBDL", MAPPING::RBDL}, {"CUSTOM", MAPPING::CUSTOM}};
const std::vector<std::string> controllers = {"custom_controller/ActuatorPositionController"};

const int NON_EXISTING = INT_MAX;
const unsigned int RBDL_NON_EXISTING = UINT_MAX;

}
}
#endif
