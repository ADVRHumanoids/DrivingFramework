#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/robot_points/minus.h"
#include "mwoibn/common/update_manager.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 ********to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPoint : public BasicTask
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 ********controlled by this task instance it makes a local copy of a point handler to
 ********prevent outside user from modifying a controlled point
 *
 */
ContactPoint( mwoibn::robot_points::Point& base_point)
        : BasicTask(), base(base_point)
{}

virtual void subscribe(bool error, bool jacobian, bool state) = 0;

virtual void unsubscribe(bool error, bool jacobian, bool state) = 0;


virtual void start() = 0;

virtual void reset() = 0;


virtual void updateState() = 0;

  virtual void updateError() = 0;
  virtual void updateJacobian() = 0;


  virtual const mwoibn::VectorN& getReference() const = 0;

  virtual void setReference(const mwoibn::VectorN& reference) = 0;

  virtual mwoibn::VectorN getReference(int i) const = 0;

  virtual void setReference(int i, const mwoibn::Vector3& reference) = 0;


  virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference,
                                   bool update) = 0;

  virtual mwoibn::Vector3 getReferenceWorld(int i)  = 0;
  virtual mwoibn::Vector3 getCurrentWorld(int i)  = 0;

  virtual const mwoibn::Vector3& getVelocityReference(int i) = 0;
  virtual const mwoibn::Vector3& getPointStateReference(int i) = 0;

  virtual const mwoibn::Vector3& getReferenceError(int i) = 0;

virtual const mwoibn::VectorN& getFullError() = 0;

virtual const mwoibn::VectorN& getWorldError() const  = 0;

virtual const mwoibn::VectorN& getForce() = 0;

virtual void releaseContact(int i)  = 0;
virtual void claimContact(int i)  = 0;

virtual void setVelocity(mwoibn::VectorN& velocity) = 0;


virtual double heading() = 0;

virtual double baseX() = 0;

virtual double baseY() = 0;

protected:
  virtual void _updater(std::shared_ptr<mwoibn::update::Function>& func) = 0;

  //! generic function to provide the same syntax for Jacobian update of all derived classes
  virtual void _updateJacobian() = 0;
  //! generic function to provide the same syntax for error update of all derived classes
  virtual void _updateError() = 0;

  virtual mwoibn::Vector3 _worldToBase(mwoibn::Vector3 point) = 0;

  virtual mwoibn::Vector3 _baseToWorld(mwoibn::Vector3 point) = 0;

  virtual void _allocate() = 0;

  virtual mwoibn::Vector3 _referencePoint(int i) = 0;


  virtual void _updateState() = 0;

public:
  const mwoibn::robot_points::Point& base;





};
}
} // namespace package
} // namespace library
#endif
