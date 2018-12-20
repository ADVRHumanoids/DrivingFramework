#ifndef __MWOIBN__COMMON__STATE_H
#define __MWOIBN__COMMON__STATE_H

#include "mwoibn/common/types.h"
#include "mwoibn/common/interfaces.h"
#include "mwoibn/common/pipe.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{

class State
{

public:
State(int dofs = 0): _interfaces({{"POSITION", mwoibn::robot_class::Pipe()},
                                 {"VELOCITY", mwoibn::robot_class::Pipe()},
                                 {"TORQUE", mwoibn::robot_class::Pipe()},
                                 {"ACCELERATION", mwoibn::robot_class::Pipe()},
                                 {"ZERO", mwoibn::robot_class::Pipe()},}),
                      position(_interfaces["POSITION"]),
                      velocity(_interfaces["VELOCITY"]),
                      torque(_interfaces["TORQUE"]),
                      acceleration(_interfaces["ACCELERATION"]),
                      zero(_interfaces["ZERO"])
{
        init(dofs);
}

State(State& other): _interfaces(other._interfaces),
                      position(_interfaces["POSITION"]),
                      velocity(_interfaces["VELOCITY"]),
                      torque(_interfaces["TORQUE"]),
                      acceleration(_interfaces["ACCELERATION"]),
                      zero(_interfaces["ZERO"])
{
}


State(State&& other): _interfaces(other._interfaces),
                      position(_interfaces["POSITION"]),
                      velocity(_interfaces["VELOCITY"]),
                      torque(_interfaces["TORQUE"]),
                      acceleration(_interfaces["ACCELERATION"]),
                      zero(_interfaces["ZERO"])
{
}

bool add(mwoibn::Interface interface){
  if(_interfaces.count(interface))
    return false;

  _interfaces[interface] = mwoibn::robot_class::Pipe(position.size());
  return true;
}

bool add(mwoibn::Interface interface, int size){
  if(_interfaces.count(interface))
    return false;

  _interfaces[interface] = mwoibn::robot_class::Pipe(size);
  return true;
}


bool has(mwoibn::Interface interface){
  return _interfaces.count(interface);
}

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
      for(auto& interface: _interfaces )
        synch(other, interface.first);
}

bool synch(State& other, mwoibn::Interface interface){
      if(!other.has(interface) ||  !has(interface)) return false;
      _interfaces[interface] = other[interface];

      return true;
}

void init(int dofs){
    for(auto& interface: _interfaces)
      interface.second.init(dofs);
}

const mwoibn::robot_class::Pipe& operator[] (mwoibn::Interface interface) const {
      return _interfaces.at(interface);
}
mwoibn::robot_class::Pipe& operator[] (mwoibn::Interface interface) {
      return _interfaces.at(interface);
}



private:
  std::map<mwoibn::Interface, mwoibn::robot_class::Pipe> _interfaces;


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
