#ifndef COMMUNICATION_MODULES_BASIC_FEEDBACK_H
#define COMMUNICATION_MODULES_BASIC_FEEDBACK_H

#include "mwoibn/communication_modules/basic_module.h"

// #include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class BasicFeedback : public BasicModule
{

public:
  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap& map, bool position, bool velocity,
                bool torque)
      : BasicModule(command, map, position, velocity, torque, true)
  {  }

  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap& map, YAML::Node config)
      : BasicModule(command, map, true, config)
  {  }

  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap&& map, bool position, bool velocity,
                bool torque)
      : BasicModule(command, map, position, velocity, torque, true)
  {  }

  BasicFeedback(mwoibn::robot_class::State& command,
                mwoibn::robot_class::BiMap&& map, YAML::Node config)
      : BasicModule(command, map, true, config)
  {  }


  BasicFeedback(BasicFeedback& other)
      : BasicModule(other) {  }

  BasicFeedback(BasicFeedback&& other)
          : BasicModule(other) {  }


  virtual ~BasicFeedback() {}

  virtual mwoibn::VectorInt map() const {return _map.get();}


  virtual bool update() { run(); }
};
}
}

#endif // BASIC_CONTROLLER_H
