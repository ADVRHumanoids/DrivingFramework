#ifndef COMMUNICATION_MODULES_BASIC_FEEDBACK_H
#define COMMUNICATION_MODULES_BASIC_FEEDBACK_H

#include "mwoibn/communication_modules/basic_module.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicFeedback : public BasicModule
{

public:
  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap map, bool position, bool velocity,
                bool torque)
      : BasicModule(command, map, position, velocity, torque, true)
  {  }

  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap map, YAML::Node config)
      : BasicModule(command, map, true, config)
  {  }

  virtual ~BasicFeedback() {}

  mwoibn::VectorInt getSelector() const { return _map.get(); }
  virtual bool get() = 0;


  virtual bool update() { get(); }
};
}
}

#endif // BASIC_CONTROLLER_H
