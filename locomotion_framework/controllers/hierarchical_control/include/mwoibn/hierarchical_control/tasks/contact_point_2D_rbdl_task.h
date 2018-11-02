#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_2D_RBDL_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_2D_RBDL_TASK_H

#include "mwoibn/hierarchical_control/tasks/contact_point_rbdl_task.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 ***to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPoint2DRbdl : public ContactPointRbdl
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 ***controlled by this task instance it makes a local copy of a point handler to
 ***prevent outside user from modifying a controlled point
 *
 */
ContactPoint2DRbdl(point_handling::PositionsHandler ik,
                   mwoibn::robot_class::Robot& robot)
        : ContactPointRbdl(ik, robot)
{
        _init(_ik.size(), _ik.getFullJacobianCols());

        _buildModel();
        _reference.setZero(_ik.size() * 2);
        _directions.setZero(_ik.size() * 2);
        _height.setZero(_ik.size());

        _rotation.setZero(2, 2);

        _full_error.setZero(ik.size() * 2);
}

virtual ~ContactPoint2DRbdl() {
}

virtual void init()
{

        updateState();

        for (int i = 0; i < _ik.size(); i++)
        {
                _temp_point = _ik.getPointStateWorld(i);
                setReferenceWorld(i, _temp_point, false);
                _height[i] = _temp_point[2];
        }
}

virtual void updateState()
{
        // update state
        ContactPointRbdl::updateState();
        RigidBodyDynamics::UpdateKinematics(_flat_model, _state, _zero, _zero);

        for (int i = 0; i <  _ik.size(); i++)
        {
                mwoibn::Vector3 axis, direction;
                axis << 0, 0, 1;
                direction = _wheels_ptr->point(i).getRotationWorld().col(2); // z axis, for our kinematics wheel axis in general

                direction = direction.cross(axis); //?
                direction.normalize();

                _directions.segment<2>(2*i) = direction.head(2);
        }
}

using CartesianWorld::getReference;
using CartesianWorld::setReference;

virtual mwoibn::VectorN getReference(int i) const
{
        return _reference.segment(i * 2, 2);
}

virtual void setReference(int i, const mwoibn::Vector3& reference)
{
        _reference.segment(i * 2, 2) = reference.head<2>();
}


virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference,
                               bool update)
{
        if (update)
                updateState();

        _reference.segment(i * 2, 2) =
                RigidBodyDynamics::CalcBaseToBodyCoordinates(
                        _flat_model, _state, _ids[0], reference, false).head(2);
}

virtual mwoibn::Vector3
getReferenceWorld(int i)   // it can have update as it uses RBDL
// call and cannot be constant anyway
{

        mwoibn::Vector3 reference;
        reference.head(2) = _reference.segment(i * 2, 2);
        reference[2] = _height[i];
        return RigidBodyDynamics::CalcBodyToBaseCoordinates(
                       _flat_model, _state, _ids[0], reference, false);
}

virtual const mwoibn::Vector3& getReferenceError(int i)
{
        _rotation << std::cos(_state[2]), std::sin(_state[2]), 0,
        -std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;
        _temp_point.head<2>() = _full_error.segment<2>(2 * i);
        _temp_point[2] = _height[i];
        _point.noalias() = _rotation * _temp_point;
        return _point;
}

virtual const mwoibn::Vector3& getPointStateReference(int i) = 0;

virtual int getFullTaskSize(){
        return _full_error.size();
}

protected:
  RigidBodyDynamics::Model _flat_model;

mwoibn::VectorN _height, _full_error, _directions;

mwoibn::Matrix _rotation;
mwoibn::Vector3 _point;


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
}
} // namespace package
} // namespace library
#endif
