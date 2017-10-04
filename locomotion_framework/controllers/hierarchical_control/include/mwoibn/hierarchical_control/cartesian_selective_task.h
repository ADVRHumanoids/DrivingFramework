#ifndef HIERARCHICAL_CONTROL_CARTESIAN_SELECTIVE_TASK_H
#define HIERARCHICAL_CONTROL_CARTESIAN_SELECTIVE_TASK_H
#include "mwoibn/hierarchical_control/cartesian_world_task.h"

namespace mwoibn
{
namespace hierarchical_control
{

class CartesianSelectiveTask : public CartesianWorldTask
{

public:
  CartesianSelectiveTask(point_handling::PositionsHandler ik,
                         mwoibn::VectorN selector)
      : CartesianWorldTask(ik)
  {
    if (selector.size() != _reference.size())
    {
      std::stringstream errMsg;
      errMsg << "Wrong size of selector expected '" << _reference.size()
             << ", received " << selector.size();
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    _selector = selector;
  }

  virtual void updateError()
  {
    CartesianWorldTask::updateError();
    _error.noalias() = _error.cwiseProduct(_selector);
  }

  virtual void updateJacobian()
  {
    CartesianWorldTask::updateJacobian();

    for(int i = 0; i < _jacobian.cols(); i++)
      _jacobian.col(i) = _jacobian.col(i).cwiseProduct(_selector);

  }

  void updateSelection(int i, double value)
  {
    if (i < _selector.size() && i >= 0)
      _selector[i] = value;
  }

  virtual ~CartesianSelectiveTask() {}

protected:
  /** Defines the importance of this task for each of the rbdl dofs, tha values
   * should be between <0,1>, size of the vector has to be of the size of robot
   * dofs */
  mwoibn::VectorN _selector;

};

} // namespace package
} // namespace library

#endif // HIERARCHICAL_CONTROL_CARTESIAN_SELECTIVE_TASK_H
