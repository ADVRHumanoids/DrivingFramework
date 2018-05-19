#ifndef CONTACT_POINT_TRACKING_TASK_H
#define CONTACT_POINT_TRACKING_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/cartesian_world_task.h"

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPointTrackingTask : public CartesianWorldTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPointTrackingTask(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : CartesianWorldTask(ik), _robot(robot)
  {
  }

  virtual ~ContactPointTrackingTask() {}

  virtual void init() = 0;


  virtual void updateState() = 0;

  virtual mwoibn::VectorN getReference(int i) const = 0;

  virtual void setReference(int i, const mwoibn::Vector3& reference) = 0;

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual double getTwist() const {return 0;}
  virtual mwoibn::Vector3 twistTransform(mwoibn::Vector3 vec) const {
    return vec;
  }

  virtual mwoibn::Vector3 twistReference(int i){

    mwoibn::Vector3 reference;
    reference = _reference.segment(i * 3, 3);

    return reference;
  }
  virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference, bool update) = 0;

  virtual mwoibn::Vector3
  getReferenceWorld(int i) = 0;

  virtual mwoibn::Vector3
  getTestReference(int i){
//    std::cout << "getTestReference 1\t" << _getTransform() << std::endl;

//    std::cout << "getTestReference 2\t" << _getTransform()*_reference.segment(i * 3, 3) << std::endl;
    mwoibn::Vector3 test = _getTransform()*_reference.segment(i * 3, 3);
    return test;
  }

  virtual const mwoibn::Vector3& getPointStateReference(int i) = 0;

  const mwoibn::VectorN& getState() const { return _state; }
  const mwoibn::VectorN& getWorldError() const { return _error; }

  virtual const mwoibn::Vector3& getReferenceError(int i) = 0;

  virtual void releaseContact(int i) { }
  virtual void claimContact(int i) { }


protected:
  mwoibn::VectorN _state;
  mwoibn::robot_class::Robot& _robot;
  mwoibn::Matrix3 _transform;
  virtual const mwoibn::Matrix3& _getTransform(){
    return _transform;
  }
  //mwoibn::Vector3 _point_flat, _temp_point;
  //mwoibn::Matrix _jacobian_3D, _jacobian_flat_3D, _jacobian_flat_2D, _rotation,
  //    _jacobian6, _jacobian_2D;
  //mwoibn::PseudoInverse _inverser;
};
} // namespace package
} // namespace library
#endif
