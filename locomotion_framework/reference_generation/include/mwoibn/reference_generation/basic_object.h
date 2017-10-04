#ifndef REFERENCE_GENERATION_BASIC_OBJECT_H
#define REFERENCE_GENERATION_BASIC_OBJECT_H

#include "mwoibn/reference_generation/reference_generation.h"
#include "mwoibn/reference_generation/utils.h"
#include <Eigen/Dense>


// Temporary include
#include <ros/ros.h>


/** This is a virtual call providing an interface for a generig reference class
 * **/
namespace mwoibn{
namespace reference_generation
{

template <typename State> class BasicObject
{

public:
  BasicObject() {}
  virtual ~BasicObject() {}

  /** @brief move one step forward on trajectory **/
  virtual Eigen::VectorXd nextStep() = 0;
  /** @brief move one step back on trajectory **/
  virtual Eigen::VectorXd backStep() = 0;

  /** @brief returns current expected final state **/
  virtual State getFinalState() { return _final_state; }
  /** @brief changes desired final state **/
  virtual void setFinalState(State new_final_state)
  {
    _final_state = new_final_state;
  }
  /** @brief returns start point state **/
  virtual State getOriginState() { return _origin_state; }
  /** @brief returns current position on a trajectory **/
  virtual State getCurrentState() { return _current_state; }

  /** @brief returns current expected final state **/
  virtual Eigen::VectorXd getFinalPoint() = 0;
//  /** @brief changes desired final state **/
//  virtual void setFinalPoint(Eigen::VectorXd new_final_state) = 0;
  /** @brief returns start point state **/
  virtual Eigen::VectorXd getOriginPoint() = 0;
  /** @brief returns current position on a trajectory **/
  virtual Eigen::VectorXd getCurrentPoint()  = 0;


  /** @brief returns size of a step on a trajectory **/
  virtual double getStep() { return _step; }
  /** @brief cahnge size of a step on a trajectory **/
  virtual void setStep(double new_step) { _step = new_step; }

  /** @brief sets a current state as a new origin **/
  virtual void resetOrigin() { _origin_state = _current_state; }
  virtual bool isDone() { return _final_state == _current_state; }

protected:
  State _final_state;
  State _origin_state;
  double _step;
  State _current_state;
};
} // namespace package
} // namespace library
#endif // BASIC_OBJECT_H
