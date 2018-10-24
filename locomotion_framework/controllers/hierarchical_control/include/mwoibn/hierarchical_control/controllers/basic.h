#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_BASIC_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_BASIC_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/eigen_utils/eigen_utils.h"
#include "mwoibn/basic_controllers/basic_controller.h"
#include <functional>

namespace mwoibn {
namespace hierarchical_control {
namespace controllers {

class Basic : public mwoibn::basic_controllers::BasicController
{
public:
Basic() : mwoibn::basic_controllers::BasicController() {
}

virtual ~Basic() {
}
virtual void init(){
}

virtual void addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
                     double damping = 1e-8) = 0;
virtual void addTask(tasks::BasicTask& new_task, double gain,
                     double damping = 1e-8) = 0;
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain,
                     double damping = 1e-8) = 0;
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, double gain,
                     double damping = 1e-8) = 0;


//! Removes a task from the stack
virtual void removeTask(unsigned int i) = 0;


virtual const mwoibn::VectorN& update() = 0;
//! computes the controll law without updating the controllers states
/**
 * \see Basic#updateController()
 *
 */
virtual void compute() = 0;

/** @brief Allows to modify controller gains online **/
//virtual bool updateGain(int i, mwoibn::VectorN gain) = 0;

//virtual bool updateGain(int i, double gain) =0;

virtual int size() = 0;

};
} // namespace package
} // namespace library
}

#endif
