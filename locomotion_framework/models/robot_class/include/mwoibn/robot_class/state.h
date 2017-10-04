#ifndef ROBOT_CLASS_STATE_H
#define ROBOT_CLASS_STATE_H

#include "mwoibn/robot_class/robot_class.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{

class State
{

public:
  State(int dofs = 0)
  {
    _init(dofs);
  }

  /** @resizes the states, it will reset to zero all values stored in an object */
  void restart(int dofs){
    _init(dofs);
  }


  void set(const mwoibn::VectorN& new_state,
      const INTERFACE interface = INTERFACE::POSITION)
  {
    for(int i = 0; i< _state(interface).size(); i++)
    _state(interface)[i] = new_state[i];
  }

  template <typename Vector>
  void set(const Vector& new_state,
      const INTERFACE interface = INTERFACE::POSITION)
  {
    mwoibn::VectorN state = _state(interface);
    for (int i = 0; i < new_state.size(); i++)
      state[i] = new_state[i];
    _state(interface) = state;
  }

  const mwoibn::VectorN& get(INTERFACE interface = INTERFACE::POSITION) const
  {
    return state(interface);
  }

  template <typename Vector, typename Selector>
  void
  get(Vector& state, const Selector& selector,
      const INTERFACE interface = INTERFACE::POSITION)
  {
    _select(_state(interface), state, selector);
  }

  template <typename Vector, typename Selector>
  void
  set(const Vector& state, const Selector& selector,
      const INTERFACE interface = INTERFACE::POSITION)
  {
     _select(state, _state(interface), selector);
  }

  const mwoibn::VectorN& state(INTERFACE interface) const;

  ~State() {}

protected:
  mwoibn::VectorN _position;
  mwoibn::VectorN _velocity;
  mwoibn::VectorN _torque;

  mwoibn::VectorN& _state(const INTERFACE interface);

  template<typename Vector1, typename Vector2>
  void _select(const Vector1& new_state, Vector2& state, mwoibn::VectorBool selector)
  {
    for (int i = 0; i < selector.size(); i++)
      if (selector[i])
        state[i] = new_state[i];
  }

  template<typename Vector, typename Vector2>
  void _select(const Vector& new_state, Vector2& state, std::vector<int> selector)
  {
    for (int i = 0; i < selector.size(); i++)
      if (selector[i] != NON_EXISTING)
        state[selector[i]] = new_state[i];
  }

  template<typename Vector, typename Vector2>
  void _select(const Vector& new_state, Vector2& state, const mwoibn::VectorInt& selector)
  {

    for (int i = 0; i < selector.size(); i++)
      if (selector[i] != NON_EXISTING)
        state[selector[i]] = new_state[i];
  }

  void _init(int dofs){
    _position = mwoibn::VectorN::Zero(dofs);
    _velocity = mwoibn::VectorN::Zero(dofs);
    _torque = mwoibn::VectorN::Zero(dofs);
  }
};
}
}

#endif // STATE_H
