#ifndef BASIC_CONTROLLERS_BASIC_CONTROLLER_H
#define BASIC_CONTROLLERS_BASIC_CONTROLLER_H

#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot_class.h"

namespace mwoibn{
namespace basic_controllers {

class BasicController{

public:
  BasicController(){}
  virtual ~BasicController(){}

/** @brief returns last computed command of the controller*/
  const mwoibn::VectorN& getCommand(){return _command;}

  /** @brief computes controller law implemented by the controller*/
  virtual void compute() = 0;

  /** @brief updates the whole controller and returnes last computed command*/
  virtual const mwoibn::VectorN& update(){
    compute();
    return getCommand();
  }

protected:
  /** Keeps last computed command of the controller **/
  mwoibn::VectorN _command;

};

} // namespace package
} // namespace library

#endif
