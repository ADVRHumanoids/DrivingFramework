#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CARTESIAN_REFERENCE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CARTESIAN_REFERENCE_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/cartesian_world_task.h"
//#include <rbdl/rbdl.h>
//#include "point_handling/points_handler.h"
#include "mwoibn/point_handling/robot_points_handler.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{
/**
 * @brief The CartesianReference class provides the implementation of a
 **CartesianWorld when a reference is not given in a world frame
 *
 */
class CartesianReference : public CartesianWorld
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 **controlled by this task instance it makes a local copy of a point handler to
 **prevent outside user from modifying a controlled point
 *
 */
CartesianReference(point_handling::PositionsHandler ik,
                   std::string reference_body,
                   mwoibn::robot_class::Robot& robot)
        : CartesianWorld(ik),
        _reference_handler(point_handling::PositionsHandler("ROOT", robot))
{
        for (int i = 0; i < _ik.size(); i++)
        {
                _reference_handler.addPoint(reference_body);
                _reference_handler.setPointStateWorld(i,_ik.getPointStateWorld(i));
        }
}

/**
 * @param[in] ik the point handler mamber that defines which point is
 **controlled by this task instance it makes a local copy of a point handler to
 **prevent outside user from modifying a controlled point
 *
 */
CartesianReference(point_handling::PositionsHandler ik,
                   int reference_body, mwoibn::robot_class::Robot& robot)
        : CartesianWorld(ik),
        _reference_handler(
                point_handling::PositionsHandler(reference_body, robot))
{
        for (int i = 0; i < _ik.size(); i++)
        {
                _reference_handler.addPoint(reference_body);
                _reference_handler.setPointStateWorld(i,_ik.getPointStateWorld(i));
        }
}
virtual ~CartesianReference() {
}

/** updates task error based on the current state of the robot and task
   /* reference position
 */
virtual void updateError()
{
        _reference = _reference_handler.getFullStateWorld();
        CartesianWorld::updateError();
}

//! sets task reference
virtual void setReference(mwoibn::VectorN reference)
{
        _reference_handler.setFullStateFixed(reference);
}
//! sets task reference
virtual void setReference(int id, mwoibn::VectorN reference)
{
        _reference_handler.setPointStateFixed(id, reference);
}
//! returnes task reference
virtual const mwoibn::VectorN& getReference()
{
        return _reference_handler.getFullStateFixed();
}
//! returnes task reference
virtual mwoibn::VectorN getReference(int i)
{
        return _reference_handler.getPointStateFixed(0);
}

const point_handling::PositionsHandler& referenceHandler()
{
        return _reference_handler;
}

//! sets current state as a desired reference
virtual void resetReference()
{
        _reference_handler.setFullStateWorld(_ik.getFullStateWorld());
}

protected:
//!point handler for a reference
point_handling::PositionsHandler _reference_handler;
void _init(){

}
};
}
} // namespace package
} // namespace library
#endif
