#ifndef DYANMIC_MODELS_BASIC_MODEL_H
#define DYANMIC_MODELS_BASIC_MODEL_H

#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/dynamic_models/dynamic_models.h"

namespace mwoibn
{

namespace dynamic_models
{

/** @brief This class provides a base for all dynamic models
 *
 * It directly implements RBDL methods for a robot model.
 *
 **/
class BasicModel
{

public:
  BasicModel(mwoibn::robot_class::Robot& robot) : _robot(robot) {

    _zero.setZero(_robot.getDofs());
    _gravity.setZero( _robot.getDofs());
    _non_linear.setZero(_robot.getDofs());
    _inertia.setZero(_robot.getDofs(),
                     _robot.getDofs());

  }

  virtual ~BasicModel() {
  }

  /** @brief returns gravity effect computed **/
  virtual const mwoibn::VectorN& getGravity()
  {

    _gravity.setZero();
    RigidBodyDynamics::NonlinearEffects(_robot.getModel(),
                                        _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION), _zero, _gravity);

    return _gravity;
  }

  /** @brief returns all modeled nonlinear effects including gravity in robots
   * dynamic **/
  virtual const mwoibn::VectorN& getNonlinearEffects()
  {

    _non_linear.setZero();
    RigidBodyDynamics::NonlinearEffects(_robot.getModel(),
                                        _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                                        _robot.state.get(mwoibn::robot_class::INTERFACE::VELOCITY), _non_linear);

    return _non_linear;
  }

  /** @brief returns inertia matrix**/
  virtual const mwoibn::Matrix& getInertia()
  {
    _inertia.setZero();

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(
        _robot.getModel(), _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION), _inertia, false);

    return _inertia;
  }

  virtual void update() {}

  mwoibn::robot_class::Robot& getRobot() const { return _robot; }

protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::VectorN _gravity, _non_linear, _zero;
  mwoibn::Matrix _inertia;


};
} // namespace package
} // namespace library

#endif // DYANMIC_MODELS_BASIC_MODEL_H
