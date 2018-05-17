#ifndef CONTACT_POINT_RBDL_TASK_H
#define CONTACT_POINT_RBDL_TASK_H

#include "mwoibn/hierarchical_control/contact_point_tracking_task.h"
#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPointRbdlTask : public ContactPointTrackingTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPointRbdlTask(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : ContactPointTrackingTask(ik, robot)
  {
    _pelvis_ptr.reset(new mwoibn::point_handling::PositionsHandler(
        "ROOT", robot, robot.getLinks("base")));
    _wheels_ptr.reset(
          new mwoibn::point_handling::OrientationsHandler("ROOT", robot, robot.getLinks("wheels")));

    _buildModel();
    _state.setZero(6);
    _zero.setZero(6);
  }

  virtual ~ContactPointRbdlTask() {}

  virtual void init() = 0;


  virtual void updateState(){

    _temp_point = _pelvis_ptr->getPointStateWorld(0);
    _state[0] = _temp_point[0];
    _state[1] = _temp_point[1];
    _state[3] = _temp_point[2];

    getAngles(_pelvis_ptr->point(0)
                      .getRotationWorld(_robot.state.get(
                          mwoibn::robot_class::INTERFACE::POSITION)));

    _state[2] = _temp_point[0];
    _state[4] = _temp_point[1];
    _state[5] = _temp_point[2];

    RigidBodyDynamics::UpdateKinematics(_flat_model, _state, _zero, _zero);

  }

  virtual void getAngles(const mwoibn::Matrix3& rotation)
  {

    _temp_point = rotation.transpose().eulerAngles(2, 1, 0);

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
  RigidBodyDynamics::Model _flat_model;
  std::unique_ptr<mwoibn::point_handling::PositionsHandler> _pelvis_ptr;
  std::unique_ptr<mwoibn::point_handling::OrientationsHandler> _wheels_ptr;

  mwoibn::VectorN _zero;
  mwoibn::Vector3 _temp_point;
  std::vector<int> _ids;

  void _buildModel(){

    _flat_model.gravity = mwoibn::Vector3(0., -9.81, 0.);
    RigidBodyDynamics::Math::Matrix3d inertia =
        RigidBodyDynamics::Math::Matrix3dZero;
    RigidBodyDynamics::Math::Vector3d com =
        RigidBodyDynamics::Math::Vector3dZero;

    RigidBodyDynamics::Body body_a(0, com, inertia);
    RigidBodyDynamics::Joint joint_a(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 1., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 1., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 1., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        0, RigidBodyDynamics::Math::Xtrans(
               RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_a, body_a));

    RigidBodyDynamics::Body body_b(0, com, inertia);
    RigidBodyDynamics::Joint joint_b(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 0., 1.),
        RigidBodyDynamics::Math::SpatialVector(0., 1., 0., 0., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(1., 0., 0., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        _ids.back(), RigidBodyDynamics::Math::Xtrans(
                         RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_b, body_b));
  }
};
} // namespace package
} // namespace library
#endif
