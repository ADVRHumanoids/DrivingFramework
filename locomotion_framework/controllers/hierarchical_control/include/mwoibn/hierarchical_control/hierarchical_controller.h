#ifndef HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_H
#define HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROLLER_H

#include <rbdl/rbdl.h>
#include "mwoibn/hierarchical_control/controller_task.h"
#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/eigen_utils/eigen_utils.h"
#include "mwoibn/basic_controllers/basic_controller.h"

// temporary libraries
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
/**
 * \todo change the package name to hierarchical_control
 * \todo add to the naming convention do not name the package and internal class
 *the same
 *
 */
namespace mwoibn{
namespace hierarchical_control
{

//! Implementation of a hierarchical controller
/**
 * \todo add a reference to the paper about the implemented controller
 * \todo add task names for convenience
 * \todo add some utility functions, so that the user can get some information
 *about tasks (combined with the name utility)
 * \todo add basic cartesian position task
 * \todo add a constructor taking the tasks (with variable number of arguments)
 * \todo **extract the continous adjustement to the task change as an inheriting
 *class**
 *
 */
class HierarchicalController : public mwoibn::basic_controllers::BasicController
{
public:
  HierarchicalController() : mwoibn::basic_controllers::BasicController() { }
  virtual ~HierarchicalController() {}

  /**
  * \brief The addition of a task to the stuck
  *
  * @param[in] i if defined the controller will add the task in a specified
  *place task[i] pushing back lower priorty tasks, otherwise a task is added at
  *the and of a stack.
  *
  * \note for the size-variable tasks the gain needs to be initialized by the
  *vector with maximum number of gains
  * \todo improve handling of size-variable tasks (adjust the gains size)
  */
  void addTask(ControllerTask* new_task, mwoibn::VectorN gain,
               int i = -1, double damping = 1e-8);
  //! Removes a task from the stack
  void removeTask(int i);

  /**
* \see HierarchicalController#compute()
   *
   */
  virtual const mwoibn::VectorN& update();
  //! computes the controll law without updating the controllers states
  /**
   * \see HierarchicalController#updateController()
   *
   */
  virtual void compute();

  /** @brief Allows to modify controller gains online **/
  bool updateGain(int i, mwoibn::VectorN gain)
  {
    _gains.at(i).noalias() = gain;
    return true;
  }
  bool updateGain(int i, double gain)
  {
    _gains.at(i).noalias() =
        gain * mwoibn::VectorN::Ones(_gains.at(i).size());
    return true;
  }
//  bool updateDamping(int i, double damp)
//  {
//    _damping.at(i) = damp;
//    return true;
//  }

  int size(){return _tasks_ptr.size()+1;}

protected:
  /** \brief Comprises pointers to all of the tasks,
   * in the priority order.
   *
   * The lower position in the stack the higher priority (i.e tasks[0] is more
   *important than tasks[1])
   */
  std::vector<ControllerTask*> _tasks_ptr;
  //! keeps tasks gains
  std::vector<mwoibn::VectorN> _gains;
  std::vector<mwoibn::VectorN> _errors;
  std::vector<std::unique_ptr<mwoibn::Projection>> _inversers_ptrs;

  mwoibn::Matrix _P;

  double _dofs;
};
} // namespace package
} // namespace library

#endif
