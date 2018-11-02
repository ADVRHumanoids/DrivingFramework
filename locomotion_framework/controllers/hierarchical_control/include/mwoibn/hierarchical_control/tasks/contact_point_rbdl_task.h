#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_RBDL_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_RBDL_TASK_H

#include "mwoibn/hierarchical_control/tasks/contact_point_tracking_task.h"
#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPointRbdl : public ContactPointTracking
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPointRbdl(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : ContactPointTracking(ik, robot)
  {
    _pelvis_ptr.reset(new mwoibn::point_handling::PositionsHandler(
        "ROOT", robot, robot.getLinks("base")));
    _wheels_ptr.reset(
          new mwoibn::point_handling::OrientationsHandler("ROOT", robot, robot.getLinks("wheels")));


    _state.setZero(6);
    _zero.setZero(6);
  }

  virtual ~ContactPointRbdl() {}

  virtual void init() = 0;


  virtual void updateState(){

    _temp_point = _pelvis_ptr->getPointStateWorld(0);
    _state[0] = _temp_point[0];
    _state[1] = _temp_point[1];
    _state[3] = _temp_point[2];

    getAngles(_pelvis_ptr->point(0).getRotationWorld());

    _state[2] = _temp_point[0];
    _state[4] = _temp_point[1];
    _state[5] = _temp_point[2];

  }

  virtual void getAngles(const mwoibn::Matrix3& rotation)
  {

    _temp_point = rotation.eulerAngles(2, 1, 0);

    //    ensure the angles are in the correct ranges (-pi:pi, -pi/2:pi/2,
    //    -pi:pi)
    mwoibn::eigen_utils::wrapToPi(_temp_point[0]);
    mwoibn::eigen_utils::wrapToPi(_temp_point[1]);
    mwoibn::eigen_utils::wrapToPi(_temp_point[2]);

    if (std::fabs(_temp_point[1]) > mwoibn::HALF_PI)
    {
      _temp_point[0] -= mwoibn::PI;
      _temp_point[1] = -_temp_point[1] - mwoibn::PI;
      _temp_point[2] -= mwoibn::PI;
    }

    mwoibn::eigen_utils::wrapToPi(_temp_point[0]);
    mwoibn::eigen_utils::wrapToPi(_temp_point[1]);
    mwoibn::eigen_utils::wrapToPi(_temp_point[2]);

  }


  virtual mwoibn::VectorN getReference(int i) const = 0;

  virtual void setReference(int i, const mwoibn::Vector3& reference) = 0;

  virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference, bool update) = 0;

//  virtual mwoibn::Vector3
//  getReferenceWorld(int i) = 0;

  virtual const mwoibn::Vector3& getPointStateReference(int i) = 0;

  virtual const mwoibn::Vector3& getReferenceError(int i) = 0;

protected:
  std::unique_ptr<mwoibn::point_handling::PositionsHandler> _pelvis_ptr;
  std::unique_ptr<mwoibn::point_handling::OrientationsHandler> _wheels_ptr;

  mwoibn::VectorN _zero;
  mwoibn::Vector3 _temp_point;
  std::vector<int> _ids;
};
}
} // namespace package
} // namespace library
#endif
