#ifndef COMMUNICATION_MODULES_BASIC_CONTROLLER_H
#define COMMUNICATION_MODULES_BASIC_CONTROLLER_H

#include "mwoibn/communication_modules/basic_module.h"

#include <rbdl/rbdl.h>

namespace mwoibn {

namespace communication_modules {

class BasicController : public BasicModule {

public:
  BasicController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, bool position, bool velocity, bool torque): BasicModule(command, map, position, velocity, torque, false){}

  BasicController(mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map, YAML::Node config): BasicModule(command, map, false, config){}

  virtual ~BasicController(){}


  mwoibn::VectorInt getSelector() const {return _map.reversed();}

  virtual bool send() = 0;
  virtual bool update() {send();}

};


}

}

#endif // BASIC_CONTROLLER_H
