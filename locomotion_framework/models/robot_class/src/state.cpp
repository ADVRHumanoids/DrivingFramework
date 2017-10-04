#include "mwoibn/robot_class/state.h"

mwoibn::VectorN&
mwoibn::robot_class::State::_state(INTERFACE interface)
{
  switch (interface)
  {
  case INTERFACE::POSITION:
    return _position;
    break;
  case INTERFACE::TORQUE:
    return _torque;
    break;
  case INTERFACE::VELOCITY:
    return _velocity;
    break;
  default:
#ifdef LOGGER
    LOG_INFO << " Tried to use INTERFACE not specified in this controller"
             << std::endl;
#endif
#ifndef LOGGER
    std::cout << " Tried to use INTERFACE not specified in this controller"
              << std::endl;
#endif
  }
}

const mwoibn::VectorN&
mwoibn::robot_class::State::state(INTERFACE interface) const
{
  switch (interface)
  {
  case INTERFACE::POSITION:
    return _position;
    break;
  case INTERFACE::TORQUE:
    return _torque;
    break;
  case INTERFACE::VELOCITY:
    return _velocity;
    break;
  default:
#ifdef LOGGER
    LOG_INFO << " Tried to use INTERFACE not specified in this controller"
             << std::endl;
#endif
#ifndef LOGGER
    std::cout << " Tried to use INTERFACE not specified in this controller"
              << std::endl;
#endif
  }
}

