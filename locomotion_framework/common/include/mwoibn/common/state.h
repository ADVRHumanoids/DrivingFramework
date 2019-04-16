#ifndef __MWOIBN__COMMON__STATE_H
#define __MWOIBN__COMMON__STATE_H

#include "mwoibn/common/types.h"
#include "mwoibn/common/interfaces.h"
#include "mwoibn/common/pipe.h"
#include "mwoibn/common/stack.h"

//#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{

class State: public mwoibn::common::Stack<mwoibn::Interface, mwoibn::robot_class::Pipe>
{
  typedef mwoibn::common::Stack<mwoibn::Interface, mwoibn::robot_class::Pipe> Precedesor_;

public:
State(int dofs = 0): Precedesor_({"POSITION", "VELOCITY", "TORQUE",
                                  "ACCELERATION", "ZERO"}),
                      position(Precedesor_::_interfaces["POSITION"]),
                      velocity(Precedesor_::_interfaces["VELOCITY"]),
                      torque(Precedesor_::_interfaces["TORQUE"]),
                      acceleration(Precedesor_::_interfaces["ACCELERATION"]),
                      zero(Precedesor_::_interfaces["ZERO"])
{
        init(dofs);
}

State(const State& other): Precedesor_(other),
                      position(Precedesor_::_interfaces["POSITION"]),
                      velocity(Precedesor_::_interfaces["VELOCITY"]),
                      torque(Precedesor_::_interfaces["TORQUE"]),
                      acceleration(Precedesor_::_interfaces["ACCELERATION"]),
                      zero(Precedesor_::_interfaces["ZERO"])
{
}


State(State&& other): Precedesor_(other),
                      position(Precedesor_::_interfaces["POSITION"]),
                      velocity(Precedesor_::_interfaces["VELOCITY"]),
                      torque(Precedesor_::_interfaces["TORQUE"]),
                      acceleration(Precedesor_::_interfaces["ACCELERATION"]),
                      zero(Precedesor_::_interfaces["ZERO"])
{
}

State& operator=(const State& other){
    if(&other == this) return *this;

    for(auto& interface: other.Precedesor_::_interfaces)
        Precedesor_::_interfaces[interface.first] = interface.second;

    return *this;
}
//
// bool add(mwoibn::Interface interface){
//   if(_interfaces.count(interface))
//     return false;
//
//   _interfaces[interface] = mwoibn::robot_class::Pipe(position.size());
//   return true;
// }
//
// bool add(mwoibn::Interface interface, int size){
//   if(_interfaces.count(interface))
//     return false;
//
//   _interfaces[interface] = mwoibn::robot_class::Pipe(size);
//   return true;
// }


// bool has(mwoibn::Interface interface){
//   return _interfaces.count(interface);
// }

/** @resizes the states, it will reset to zero all values stored in an object */
void restart(int dofs){
        init(dofs);
}

virtual ~State() { }

bool synch(State& other, std::initializer_list<mwoibn::Interface> interfaces){
  bool success = true;
  for(auto& interface: interfaces )
      success = synch(other, interface) && success;

  return success;
}

void synch(State& other){
      for(auto& interface: Precedesor_::_interfaces )
        synch(other, interface.first);
}

bool synch(State& other, mwoibn::Interface interface){
      if(!other.has(interface) ||  !has(interface)) return false;
      Precedesor_::_interfaces[interface] = other[interface];

      return true;
}
//
// void init(int dofs){
//     for(auto& interface: _interfaces)
//       interface.second.init(dofs);
// }

// const mwoibn::robot_class::Pipe& operator[] (const mwoibn::Interface& interface) const {
//       return _interfaces.at(interface);
// }
// mwoibn::robot_class::Pipe& operator[] (const mwoibn::Interface& interface) {
//       return _interfaces.at(interface);
// }
//

//
// private:
//   std::map<mwoibn::Interface, mwoibn::robot_class::Pipe> _interfaces;


public:
  mwoibn::robot_class::Pipe& position;
  mwoibn::robot_class::Pipe& velocity;
  mwoibn::robot_class::Pipe& torque;
  mwoibn::robot_class::Pipe& acceleration;
  mwoibn::robot_class::Pipe& zero;
};
}
}

#endif // STATE_H
