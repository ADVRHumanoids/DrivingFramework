#ifndef HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H
#define HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/cartesian_world_task.h"
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
class CartesianSimplifiedPelvisTask : public CartesianWorldTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  CartesianSimplifiedPelvisTask(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : CartesianWorldTask(ik), _robot(robot)
  {
    _pelvis_ptr.reset(new mwoibn::point_handling::PositionsHandler(
        "ROOT", robot, robot.getLinks("base")));
    _wheels_ptr.reset(
          new mwoibn::point_handling::OrientationsHandler("ROOT", robot, robot.getLinks("wheels")));

//    for (int i = 0; i < ik.size(); i++)
//      _wheels_ptr->addPoint(ik.point(i));

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
    _init(_ik.size(), _ik.getFullJacobianCols());
    _state.setZero(6);

    _reference.setZero(_ik.size() * 2);
    _directions.setZero(_ik.size() * 2);
    _height.setZero(_ik.size());

    _jacobian_3D.setZero(_ik.getPointJacobianRows(0),
                         _ik.getFullPointJacobianCols(0));

    _jacobian_flat_3D.setZero(3, 6);
    _jacobian_2D.setZero(2 * ik.size(), _robot.getDofs());
    _jacobian_2D_bis.setZero(2, _robot.getDofs());
    _jacobian_flat_2D.setZero(_ik.size() * 2, 6);
    _inverser.init(_jacobian_flat_2D, 1e-6);
    _rotation.setZero(2, 2);
    _jacobian6.setZero(6, 6);
    _zero.setZero(6);
    //_wheels.setZero(ik.size());
    _point2D.setZero(2);
    _full_error.setZero(ik.size() * 2);
    // init the wheels orientations


    init();
  }

  virtual ~CartesianSimplifiedPelvisTask() {}

  void init()
  {

    updateState();

    for (int i = 0; i < _ik.size(); i++)
    {
      _temp_point = _ik.getPointStateWorld(i);
      setReferenceWorld(i, _temp_point, false);
      _height[i] = _temp_point[2];
    }
  }

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError()
  {
    _last_error.noalias() = _error; // save previous state
    //updateState(); // this will be updated by steering call

    //std::cout << "state\n" <<  _state << std::endl;

    for (int i = 0; i < _ik.size(); i++)
    {
      _point_flat = getPointStateReference(i);

      _rotation << std::cos( _state[2]), -std::sin( _state[2]),
          std::sin( _state[2]), std::cos( _state[2]);

      _full_error.segment(2 * i, 2).noalias() =
          _reference.segment(2 * i, 2) - _point_flat.head(2);

      _point2D.noalias() = _rotation * _full_error.segment(2 * i, 2);

      _error[i] = _directions[2*i] * _point2D[0] + _directions[2*i + 1] * _point2D[1];

    }

  }

  void updateState()
  {
    // update state

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

//    std::cout << "wheels" << std::endl;

    for (int i = 0; i <  _ik.size(); i++)
    {
      mwoibn::Vector3 axis, direction;
      axis << 0, 0, 1;
      direction = _wheels_ptr->point(i)
                           .getRotationWorld(_robot.state.get(
                               mwoibn::robot_class::INTERFACE::POSITION))
                           .row(2); // z axis

      direction = direction.cross(axis); //?
      direction.normalize();

      _directions.segment<2>(2*i) = direction.head(2);
    }
  }

  void getAngles(const mwoibn::Matrix3& rotation)
  {

    _temp_point = rotation.transpose().eulerAngles(2, 1, 0);

    //    ensure the angles are in the correct ranges (-pi:pi, -pi/2:pi/2,
    //    -pi:pi)
    _temp_point[0] -= 6.28318531 * std::floor((_temp_point[0] + 3.14159265) /
                                              6.28318531); //-pi:pi
    _temp_point[1] -= 6.28318531 * std::floor((_temp_point[1] + 3.14159265) /
                                              6.28318531); //-pi:pi
    _temp_point[2] -= 6.28318531 * std::floor((_temp_point[2] + 3.14159265) /
                                              6.28318531); //-pi:pi

    if (std::fabs(_temp_point[1]) > 1.57079633)
    {
      _temp_point[0] -= 3.14159265;
      _temp_point[1] = -_temp_point[1];
      _temp_point[2] -= 3.14159265;
    }

    _temp_point[0] -= 6.28318531 * std::floor((_temp_point[0] + 3.14159265) /
                                              6.28318531); //-pi:pi
    _temp_point[1] -= 6.28318531 * std::floor((_temp_point[1] + 3.14159265) /
                                              6.28318531); //-pi:pi
    _temp_point[2] -= 6.28318531 * std::floor((_temp_point[2] + 3.14159265) /
                                              6.28318531); //-pi:pi

//    _state[2] = _temp_point[0];
//    _state[4] = _temp_point[1];
//    _state[5] = _temp_point[2];
  }

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;
    //    _jacobian_new_state.setZero();

    for (int i = 0; i < _ik.size(); i++)
    {
      _jacobian_3D.noalias() = _ik.getFullPointJacobian(i);

      _jacobian_2D.block(2 * i, 0, 2, _robot.getDofs()) =
          _jacobian_3D.topRows<2>();

      _point_flat = RigidBodyDynamics::CalcBaseToBodyCoordinates(
          _flat_model, _state, _ids[1], _ik.getPointStateWorld(i), false);

      _jacobian_flat_3D.setZero();
      RigidBodyDynamics::CalcPointJacobian(
          _flat_model, _state, _ids[1], _point_flat, _jacobian_flat_3D, false);

      _jacobian_flat_2D.block<2, 6>(2 * i, 0) = _jacobian_flat_3D.topRows(2);
    }

    _inverser.compute(_jacobian_flat_2D);

    _jacobian6.noalias() = _inverser.get() * _jacobian_2D.leftCols(6);

    _jacobian_flat_2D.leftCols(3).setZero();

    //    _rotation << std::cos(_state[2]), std::sin(_state[2]),
    //    -std::sin(_state[2]),
    //        std::cos(_state[2]);

    _jacobian_2D.leftCols(6).noalias() = (_jacobian_flat_2D)*_jacobian6;

    for (int i = 0; i < _ik.size(); i++)
    {
      _jacobian.row(i).noalias() = - _directions.segment<2>(2*i).transpose() * _jacobian_2D.block(2 * i, 0, 2, _robot.getDofs());
    }

 //   std::cout << "_jacobian" << std::endl;
 //   std::cout << _jacobian << std::endl;
  }

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment(i * 2, 2);
  }

  virtual void setReference(int i, const mwoibn::VectorN& reference)
  {
    _reference.segment(i * 2, 2) = reference;
  }

  virtual void setReferenceWorld(int i, const mwoibn::Vector3 reference,
                                 bool update)
  {
    if (update)
      updateState();

    _reference.segment(i * 2, 2) =
        RigidBodyDynamics::CalcBaseToBodyCoordinates(
            _flat_model, _state, _ids[0], reference, false).head(2);
  }

  virtual mwoibn::Vector3
  getReferenceWorld(int i, bool update) // it can have update as it uses RBDL
                                        // call and cannot be constant anyway
  {
    if (update)
      updateState();

    mwoibn::Vector3 reference;
    reference.head(2) = _reference.segment(i * 2, 2);
    reference[2] = _height[i];
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
        _flat_model, _state, _ids[0], reference, false);
  }

  virtual const mwoibn::Vector3 getPointStateReference(int i)
  {
    return RigidBodyDynamics::CalcBaseToBodyCoordinates(
        _flat_model, _state, _ids[0], _ik.getPointStateWorld(i), false);
  }

  const mwoibn::VectorN& getState() const { return _state; }
  const mwoibn::VectorN& getWorldError() const { return _full_error; }

protected:
  mwoibn::VectorN _state, _height, _zero, _wheels, _point2D, _full_error, _directions;
  std::unique_ptr<mwoibn::point_handling::PositionsHandler> _pelvis_ptr;
  std::unique_ptr<mwoibn::point_handling::OrientationsHandler> _wheels_ptr;
  mwoibn::robot_class::Robot& _robot;
  RigidBodyDynamics::Model _flat_model;
  mwoibn::Vector3 _point_flat, _temp_point;
  std::vector<int> _ids;

  mwoibn::Matrix _jacobian_3D, _jacobian_flat_3D, _jacobian_flat_2D, _rotation,
      _jacobian6, _jacobian_2D, _jacobian_2D_bis;
  mwoibn::PseudoInverse _inverser;
};
} // namespace package
} // namespace library
#endif
