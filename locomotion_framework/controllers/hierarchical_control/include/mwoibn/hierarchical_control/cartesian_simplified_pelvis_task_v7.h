#ifndef HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_7_H
#define HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_7_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/cartesian_world_task.h"
#include "mwoibn/hierarchical_control/center_of_mass_task.h"
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
class CartesianFlatReferenceTask3 : public CartesianWorldTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  CartesianFlatReferenceTask3(point_handling::PositionsHandler ik,
                              mwoibn::robot_class::Robot& robot, mwoibn::hierarchical_control::CenterOfMassTask& com)
      : CartesianWorldTask(ik), _robot(robot), _com(com)
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
    RigidBodyDynamics::Math::Vector3d l_com =
        RigidBodyDynamics::Math::Vector3dZero;

    RigidBodyDynamics::Body body_a(0, l_com, inertia);
    RigidBodyDynamics::Joint joint_a(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 1., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 1., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 1., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        0, RigidBodyDynamics::Math::Xtrans(
               RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_a, body_a));

    RigidBodyDynamics::Body body_b(0, l_com, inertia);
    RigidBodyDynamics::Joint joint_b(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 0., 1.),
        RigidBodyDynamics::Math::SpatialVector(0., 1., 0., 0., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(1., 0., 0., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        _ids.back(), RigidBodyDynamics::Math::Xtrans(
                         RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_b, body_b));
    _init(3*_ik.size(), _ik.getFullJacobianCols());
    _state.setZero(6);

    _reference.setZero(_ik.size() * 3);
    //_directions.setZero(_ik.size() * 2);

    _rotation.setZero(3, 3);
    _zero.setZero(6);
    _jacobian_th.setZero(2, _robot.getDofs());

    _full_error.setZero(ik.size() * 3);

    //    _map = _robot.biMaps().get("full_body").get();

    _selector = mwoibn::VectorBool::Constant(
        _robot.contacts().size(),
        true); // on init assume all constacts should be considered in a task

    mwoibn::Axis axis; // for now hardcoded
    axis <<  0,  0,  1;
    _axes.push_back(axis);
    axis <<  0,  0,  -1;
    _axes.push_back(axis);
    axis <<  0,  0,  1;
    _axes.push_back(axis);
    axis <<  0,  0,  -1;
    _axes.push_back(axis);

    _axes_world = _axes;
    _x_world = _axes;

    _ground_normal << 0,0,1;
    init();
  }

  virtual ~CartesianFlatReferenceTask3() {}

  void init()
  {

    updateState();

    for (int i = 0; i < _ik.size(); i++)
    {
      _reference.segment<3>(3 * i) = getPointStateReference(i);
      //      _temp_point = _ik.getPointStateWorld(i);
      //      setReferenceWorld(i, _temp_point, false);
    }
  }

  //! updates task error based on the current state of the robot and task

  virtual void updateError()
  {
    _last_error.noalias() = _error; // save previous state
    // updateState(); // this will be updated by steering call

    for (int i = 0; i < _ik.size(); i++)
    {

      _rotation << std::cos(_state[2]), -std::sin(_state[2]), 0,
          std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;

      _full_error.segment<3>(3 * i).noalias() =
          _rotation * _reference.segment<3>(3 * i);
      _full_error.segment<3>(3 * i) -= _ik.getPointStateWorld(i) + computeContact(i);

      _full_error.segment<2>(3 * i) =
          _robot.centerOfMass().get().head(2) + _full_error.segment<2>(3 * i);

      if (_selector[i])
      {
        _error[3 * i] = _x_world[i][0] * _full_error[3 * i] +
                        _x_world[i][1] * _full_error[3 * i + 1];
        _error[3 * i + 1] = 0;
        _error[3 * i + 2] = 0;
      }
      else
      {
        _error.segment<3>(3 * i) = _full_error.segment<3>(3 * i);
        //        std::cout << "contact\t" << i << "\t" <<
        //        _full_error.segment<3>(3 * i).transpose() << "\t" <<
        //        _reference.segment<3>(3 * i).transpose() <<  std::endl;
        //        _error[3*i + 2] = 0;
      }
    }
       // std::cout << "error\t" << "\t" << _full_error.transpose() << "\t ref" << std::endl;
       // std::cout << "reference\t" << "\t" << _reference.transpose() << "\t ref" << std::endl;

    //    \t" << _reference.transpose() <<  std::endl;
  }

  mwoibn::Vector3 computeContact(int i){
    double norm = 1/(_ground_normal - _axes_world[i]*_ground_normal.transpose()*_axes_world[i]).norm();
    mwoibn::Vector3 contact = -(_ground_normal - _axes_world[i]*_ground_normal.transpose()*_axes_world[i])*norm*R;
    contact -= (_ground_normal)*r;
    //std::cout << i << "contact\t" << "\t" << contact.transpose() << "\t ref" << std::endl;

    return contact;
  }

  void updateState()
  {
    // update state

    _temp_point = _pelvis_ptr->getPointStateWorld(0);
    _state[0] = _temp_point[0];
    _state[1] = _temp_point[1];
    _state[3] = _temp_point[2];

    getAngles(_pelvis_ptr->point(0).getRotationWorld(
        _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION)));

    _state[2] = _temp_point[0];
    _state[4] = _temp_point[1];
    _state[5] = _temp_point[2];

    RigidBodyDynamics::UpdateKinematics(_flat_model, _state, _zero, _zero);

    for (int i = 0; i < _ik.size(); i++)
    {
      //mwoibn::Vector3 direction;
      _ground_normal << 0, 0, 1;
      _axes_world[i] =
          _wheels_ptr->point(i)
              .getRotationWorld(
                   _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION)).transpose()*_axes[i]; // z axis, for our kinematics wheel axis in general

      _x_world[i] = _axes_world[i].cross(_ground_normal); //?
      _x_world[i].normalize();

     // _directions.segment<2>(2 * i) = direction.head(2);
    }
  }

  void getAngles(const mwoibn::Matrix3& rotation)
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

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _ik.size(); i++)
    {
      if (_selector[i])
      {
        //computeContactJacobian(i);
        _jacobian_th.noalias() =
            -_com.getJacobian() - _ik.getFullPointJacobian(i).topRows<2>();

        //std::cout << "original\n" << _jacobian_th << std::endl;

        _jacobian_th.noalias() -= computeContactJacobian(i).topRows<2>();
        //std::cout << "after\n"  << _jacobian_th << std::endl;

        _jacobian_th(0, 2) -= (std::sin(_state[2]) * _reference[3 * i] +
                               std::cos(_state[2]) * _reference[3 * i + 1]);
        _jacobian_th(1, 2) += (std::cos(_state[2]) * _reference[3 * i] -
                               std::sin(_state[2]) * _reference[3 * i + 1]);
        _jacobian.row(3 * i).noalias() =
            _x_world[i].head<2>().transpose() * _jacobian_th;

        //_jacobian.row(3 * i).noalias() -= _x_world[i].transpose()*computeContactJacobian(i);

        _jacobian.row(3 * i + 1).setZero();
        _jacobian.row(3 * i + 2).setZero();
      }
      else
      {
        _jacobian.block(3 * i, 0, 3, _robot.getDofs()) =
            -_ik.getFullPointJacobian(i) - computeContactJacobian(i);
        _jacobian.block(3 * i, 0, 2, _robot.getDofs()) =
            -_com.getJacobian() +
            _jacobian.block(3 * i, 0, 2, _robot.getDofs());
        //        _jacobian.row(3*i + 2).setZero();
      }
    }
  }

  mwoibn::Matrix computeContactJacobian(int i){
    double norm = 1/(_ground_normal - _axes_world[i]*_ground_normal.transpose()*_axes_world[i]).norm();

    mwoibn::Vector3 vector = _axes_world[i]*_ground_normal.transpose()*_axes_world[i];
    mwoibn::Matrix3 contact_j, contact_temp;
    mwoibn::eigen_utils::skew(vector, contact_j);
    mwoibn::eigen_utils::skew(_axes_world[i], contact_temp);

    contact_j += (_axes_world[i]*_ground_normal.transpose()*contact_temp);

    mwoibn::Matrix3 contact_2 = 0.5*norm*norm*_ground_normal*_ground_normal.transpose();
    contact_2 -= mwoibn::Matrix3::Identity();
    contact_2 -= 0.5*norm*norm*_axes_world[i]*_axes_world[i].transpose()*_ground_normal*_ground_normal.transpose();

    mwoibn::Matrix contact;
    contact = contact_2*contact_j*_wheels_ptr->point(i).getOrientationJacobian(_robot.state.get(mwoibn::robot_class::INTERFACE::POSITION))*R*norm;

    return contact;
  }

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment<3>(i * 3);
  }

  virtual void setReference(int i, const mwoibn::Vector3& reference)
  {
    _reference.segment(i * 3, 3) = reference;
  }

  virtual void setReferenceWorld(int i, const mwoibn::Vector3 reference,
                                 bool update)
  {
    std::cout << "CartesianFlatReferenceTask2::setReferenceWorld" << std::endl;

    if (update)
      updateState();

    _reference.segment(i * 3, 3) = RigidBodyDynamics::CalcBaseToBodyCoordinates(
        _flat_model, _state, _ids[0], reference, false);
  }

  virtual mwoibn::Vector3
  getReferenceWorld(int i, bool update) // it can have update as it uses RBDL
                                        // call and cannot be constant anyway
  {
    if (update)
      updateState();

    mwoibn::Vector3 reference;
    reference = _reference.segment(i * 3, 3);

    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
        _flat_model, _state, _ids[0], reference, false);
  }

  virtual const mwoibn::Vector3& getPointStateReference(int i)
  {
    _rotation << std::cos(_state[2]), std::sin(_state[2]), 0,
        -std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;

    _track_point = _ik.getPointStateWorld(i) + computeContact(i);
    _track_point.head<2>() -= _robot.centerOfMass().get().head<2>();

    _point.noalias() = _rotation * _track_point;
    return _point;
  }

  const mwoibn::VectorN& getState() const { return _state; }
  const mwoibn::VectorN& getWorldError() const { return _full_error; }

  virtual const mwoibn::Vector3& getReferenceError(int i)
  {
    _rotation << std::cos(_state[2]), std::sin(_state[2]), 0,
        -std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;

    _point.noalias() = _rotation * (_full_error.segment<3>(3 * i));
    return _point;
  }

  virtual void releaseContact(int i) { _selector[i] = true; }
  virtual void claimContact(int i) { _selector[i] = false; }

protected:
  mwoibn::VectorN _state, _zero, _wheels, _full_error, _directions;
  std::unique_ptr<mwoibn::point_handling::PositionsHandler> _pelvis_ptr;
  std::unique_ptr<mwoibn::point_handling::OrientationsHandler> _wheels_ptr;
  mwoibn::hierarchical_control::CenterOfMassTask& _com;
  mwoibn::robot_class::Robot& _robot;
  RigidBodyDynamics::Model _flat_model;
  mwoibn::Vector3 _point_flat, _temp_point, _point, _track_point;
  std::vector<int> _ids;
  mwoibn::Matrix _rotation, _jacobian_th;
  mwoibn::VectorBool _selector;
  std::vector<mwoibn::Axis> _axes, _axes_world, _x_world;
  mwoibn::Axis _ground_normal;
  double R = 0.01, r = 0.068;
};
} // namespace package
} // namespace library
#endif
