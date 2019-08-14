#ifndef __MWOIBN__COMMON__PIPE_H
#define __MWOIBN__COMMON__PIPE_H

#include "mwoibn/common/types.h"
//#include "mwoibn/common/interfaces.h"

//#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{

class Pipe
{

public:
Pipe(int dofs = 0)
{
        init(dofs);
}

Pipe(const Pipe& other): _state(other._state)
{}

Pipe( Pipe&& other): _state(other._state)
{}


Pipe& operator=(const Pipe& other) {
    init(other.size());
    set(other);
    return *this;
}
/** @resizes the states, it will reset to zero all values stored in an object */
void restart(int dofs){
        init(dofs);
}


void set(const mwoibn::VectorN& new_state)
{
        for(int i = 0; i< _state.size(); i++)
                _state[i] = new_state[i];
}

template <typename Vector>
void set(const Vector& new_state)
{
        for (int i = 0; i < _state.size(); i++)
                _state[i] = new_state[i];
}

const mwoibn::VectorN& get() const
{
        return _state;
}

template <typename Vector>
void
get(Vector& state, const std::vector<int>& selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        state[i] = _state[selector[i]];

}

template <typename Vector>
void
get(Vector& state, const mwoibn::VectorInt& selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        state[i] = _state[selector[i]];
}

template <typename Vector, typename Selector>
void
get(Vector& state, const Selector& selector)
{
        _select(_state, state, selector);
}

double
get(const int selector) const
{
        return _state[selector];
}
double
get(const int selector)
{
        return _state[selector];
}
void
set(const double state, const int selector)
{
        _state[selector] = state;
}

template <typename Vector, typename Selector>
void
set(const Vector& state, const Selector& selector)
{
        _select(state, _state, selector);
}

template <typename Vector>
void
set(const Vector& state, const std::vector<int>& selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        _state[selector[i]] = state[i];
}

template <typename Vector>
void
set(const Vector& state, const mwoibn::VectorInt& selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i] != NON_EXISTING)
                        _state[selector[i]] = state[i];
}

//const mwoibn::VectorN& state() const {return _state;};

int size() const{
        return _state.size();
}

void init(int dofs){
        _state = mwoibn::VectorN::Zero(dofs);
}



Scalar operator[](int i) const {
        return _state[i];
}


virtual ~Pipe() {
}

protected:
mwoibn::VectorN _state;

template<typename Vector1, typename Vector2>
void _select(const Vector1& new_state, Vector2& state, const mwoibn::VectorBool& selector)
{
        for (int i = 0; i < selector.size(); i++)
                if (selector[i])
                        state[i] = new_state[i];
}



};
}
}

#endif // STATE_H
