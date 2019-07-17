#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_HANDLER_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_HANDLER_TASK_H

//#include <rbdl/rbdl.h>
#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/robot_points/point.h"

namespace mwoibn {

namespace hierarchical_control
{
namespace tasks
{

/**
 * \brief Basic implementation of a task in the hierarchical controller
 *
 *
 */
template<typename Point>
class HandlerTask: public BasicTask {
public:
/**
 *
 * @param[in] i number of dofs of a task
 *
 */
HandlerTask(): BasicTask(){
}

HandlerTask(HandlerTask&& other): BasicTask(other) {
}

HandlerTask(const HandlerTask& other): BasicTask(other) {
}


virtual ~HandlerTask(){
}

mwoibn::robot_points::Handler<Point> handler;
mwoibn::robot_points::Handler<mwoibn::robot_points::Point> support_points;

void init(){
  reference.clear();
  handler.update(true);
  for(auto& point_: handler) reference.push_back(point_->get());
  _init(handler.rows(), handler.cols());
}

// //! generic function to provide the same syntax for Jacobian update of all derived classes
virtual void updateJacobian(){
  for(int i = 0, rows = 0; i < handler.size(); i++){
    _jacobian.middleRows(rows, handler[i].rows()) = - handler[i].getJacobian();
    rows += handler[i].rows();
  }
}

virtual void updateError(){
    for(int i = 0, rows = 0; i < handler.size(); i++){
      _error.segment(rows, handler[i].rows()) = reference[i] - handler[i].get();
      rows += handler[i].rows();
    }
}

// //! updates whole task in one call, calls updateError() and updateJacobin() in that order
virtual void update(){
  // std::cout << "update" << std::endl;
    support_points.update(true);
    handler.update(true);
    updateError(); updateJacobian();
}
//


//! sets task reference
// Does not check for the reference size
// virtual void setReference(int i, const mwoibn::VectorN& reference)
// {
//         reference[i] = reference;
// }
//! sets current state as a desired reference
virtual void resetReference() {
        for(auto&& [reference_, point_]:  ranges::view::zip(reference, handler))
        reference_ = point_->get();
}

// //! sets task reference
// virtual const mwoibn::VectorN& getReference(int i)
// {
//         return reference[i];
// }

std::vector<mwoibn::VectorN> reference;


  // How would I know which point refers to a given point (by number?)




};
}
}     // namespace package
}     // namespace library
    #endif
