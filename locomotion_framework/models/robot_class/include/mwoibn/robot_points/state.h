#ifndef __MWOIBN__ROBOT_POINTS__STATE_H
#define __MWOIBN__ROBOT_POINTS__STATE_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

//! This class provides a specialization of the robot_points::Point interface to an element dependent on the robot rigid body kinematics
/** @param[in] model - a reference to the rigid body kinematic model (RBDL implementation)
 *  @param[in] state - a reference to the object that contains the robot state (e.g. a measured/desired state).
 */
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
  /** A robot state is a class that provides a common interface for the elements like measured/desired link-side values, measured/desired motor-side values,
   *  robot kinematic/dynamic limits. By default position, velocity, acceleration and torque interfaces are provided.
   */
  const mwoibn::robot_class::State& _state;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
