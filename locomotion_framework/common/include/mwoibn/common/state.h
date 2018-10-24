#ifndef __MWOIBN__COMMON__STATE_H
#define __MWOIBN__COMMON__STATE_H

#include "mwoibn/common/types.h"
#include "mwoibn/common/interfaces.h"

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
        for (int i = 0; i < new_state.size(); i++)
                _state(interface)[i] = new_state[i];
}

const mwoibn::VectorN& get(INTERFACE interface = INTERFACE::POSITION) const
{
        return state(interface);
}

template <typename Vector>
void
get(Vector& state, const std::vector<int> selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        state[i] = _state(interface)[selector[i]];

}

template <typename Vector>
void
get(Vector& state, const mwoibn::VectorInt& selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        state[i] = _state(interface)[selector[i]];
}

template <typename Vector, typename Selector>
void
get(Vector& state, const Selector& selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        _select(_state(interface), state, selector);
}

double
get(const int selector,
    const INTERFACE interface = INTERFACE::POSITION) const
{
        return _state(interface)[selector];
}
double
get(const int selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        return _state(interface)[selector];
}
void
set(const double state, const int selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        _state(interface)[selector] = state;
}

template <typename Vector, typename Selector>
void
set(const Vector& state, const Selector& selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        _select(state, _state(interface), selector);
}

template <typename Vector>
void
set(const Vector& state, std::vector<int> selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        _state(interface)[selector[i]] = state[i];
}

template <typename Vector>
void
set(const Vector& state, const mwoibn::VectorInt& selector,
    const INTERFACE interface = INTERFACE::POSITION)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        _state(interface)[selector[i]] = state[i];
}

const mwoibn::VectorN& state(INTERFACE interface) const;

int size() const{
        return _position.size();
}

virtual ~State() {
}

protected:
mwoibn::VectorN _position;
mwoibn::VectorN _velocity;
mwoibn::VectorN _torque;
mwoibn::VectorN _acceleration;

mwoibn::VectorN& _state(const INTERFACE interface);
const mwoibn::VectorN& _state(const INTERFACE interface) const;

template<typename Vector1, typename Vector2>
void _select(const Vector1& new_state, Vector2& state, mwoibn::VectorBool selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i])
                        state[i] = new_state[i];
}

void _init(int dofs){
        _position = mwoibn::VectorN::Zero(dofs);
        _velocity = mwoibn::VectorN::Zero(dofs);
        _torque = mwoibn::VectorN::Zero(dofs);
        _acceleration = mwoibn::VectorN::Zero(dofs);
}
};
}
}

#endif // STATE_H
