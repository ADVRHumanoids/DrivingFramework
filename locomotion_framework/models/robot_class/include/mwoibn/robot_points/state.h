#ifndef __MWOIBN__ROBOT_POINTS__STATE_H
#define __MWOIBN__ROBOT_POINTS__STATE_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

class State: public Point
{

public:
  State(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state): Point(3, state.velocity.size()), _state(state), _model(model){

  }

  State(const State& other): Point(other), _state(other._state), _model(other._model){
  }

  State( State&& other): Point(other), _state(other._state), _model(other._model){
  }


  virtual ~State() {}

protected:
  RigidBodyDynamics::Model& _model;
  const mwoibn::robot_class::State& _state;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
