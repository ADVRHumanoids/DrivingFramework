#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_ORIENTATION_WORLD_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_ORIENTATION_WORLD_TASK_H

#include "mwoibn/hierarchical_control/controllers/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include <rbdl/rbdl.h>
//#include "point_handling/points_handler.h"
#include "mwoibn/point_handling/robot_points_handler.h"

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
 * @note for now it works in a positive hemisphere of a quaternion
 */
class OrientationWorld : public BasicTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  OrientationWorld(point_handling::OrientationsHandler ik,
                       mwoibn::robot_class::Robot& robot)
      : BasicTask(), _ik(ik)
  {
    _init(_ik.getFullJacobianRows(), _ik.getFullJacobianCols());

    _reference = _ik.getFullStatesWorld();


    mwoibn::VectorN state =
        robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
    mwoibn::VectorN test = mwoibn::VectorN::Zero(robot.getDofs());

    robot.state.set(test, mwoibn::robot_class::INTERFACE::POSITION);
    robot.updateKinematics();

    _offset = _ik.getFullStatesWorld();



    robot.state.set(state, mwoibn::robot_class::INTERFACE::POSITION);
    robot.updateKinematics();

    for (int i = 0; i < _offset.size(); i++)
    {
      _offset[i].ensureHemisphere(_reference[i]);
//      _offset[i].ensureHemisphere(_previous_state[i]);
    }


  }

  virtual ~OrientationWorld() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError();
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian();
  //! sets task reference
  virtual void setReference(const std::vector<mwoibn::Quaternion>& reference)
  {
    if (reference.size() != _reference.size())
      return;
    _reference = reference;
  }
  //! sets task reference
  virtual void setReference(int i, const mwoibn::Quaternion& reference)
  {
    if (i >= _reference.size() || i < 0)
      throw std::out_of_range("given number is out of range ");
    //    _reference[i].ensureHemisphere(reference);
    _reference[i] = reference;
  }
  //! sets current state as a desired reference
  virtual void resetReference()
  {
    for (int i = 0; i < _ik.size(); i++)
    {
      setReference(i, _ik.getPointStateWorld(i));
    }
  }
  //! returnes task reference
  virtual std::vector<mwoibn::Quaternion> getReference() { return _reference; }

  const point_handling::OrientationsHandler& points() { return _ik; }
  //! sets task reference
  virtual mwoibn::Quaternion getReference(int i)
  {
    if (i >= _reference.size() || i < 0)
      throw std::out_of_range("given number is out of range");

    return _reference.at(i);
  }

  mwoibn::Quaternion getOffset(int id) const { return _offset.at(id); }
  std::vector<mwoibn::Quaternion> getOffsets() const { return _offset; }

protected:
  //!point handler mamber for point controlled by this task instance
  point_handling::OrientationsHandler _ik;
  //! task reference position of a controlled point expressed in a world frame
  std::vector<mwoibn::Quaternion> _reference;
//  std::vector<mwoibn::Quaternion> _previous_state;
  std::vector<mwoibn::Quaternion> _offset;
  mwoibn::Quaternion _current;
  mwoibn::Matrix3 _skew;
  mwoibn::Vector3 _axis;
};
}
} // namespace package
} // namespace library
#endif
