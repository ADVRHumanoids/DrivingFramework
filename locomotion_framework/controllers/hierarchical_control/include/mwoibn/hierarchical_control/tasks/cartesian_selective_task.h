#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CARTESIAN_SELECTIVE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CARTESIAN_SELECTIVE_H
#include "mwoibn/hierarchical_control/tasks/cartesian_world_task.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{
class CartesianSelective : public CartesianWorld
{

public:
  CartesianSelective(point_handling::PositionsHandler ik,
                         mwoibn::VectorN selector)
      : CartesianWorld(ik)
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
    CartesianWorld::updateError();
    _error.noalias() = _error.cwiseProduct(_selector);
  }

  virtual void updateJacobian()
  {
    CartesianWorld::updateJacobian();

    for(int i = 0; i < _jacobian.cols(); i++)
      _jacobian.col(i) = _jacobian.col(i).cwiseProduct(_selector);

  }

  void updateSelection(int i, double value)
  {
    if (i < _selector.size() && i >= 0)
      _selector[i] = value;
  }

  virtual ~CartesianSelective() {}

protected:
  /** Defines the importance of this task for each of the rbdl dofs, tha values
   * should be between <0,1>, size of the vector has to be of the size of robot
   * dofs */
  mwoibn::VectorN _selector;
};
}
} // namespace package
} // namespace library
#endif
