#ifndef __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H
#define __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/frame.h"


namespace mwoibn
{
namespace robot_points
{

//! This class provides a specialization of the robot_points::State class for the rigid body linear point geometry
class LinearPoint: public State
{

public:
 /** @param[in] Body - is a std::string with the name of the rigid body (link), or the id number (int) of the rigid body (link) in the rbdl model
 *  @param[in] robot - a reference to the robot object that contains the robot kinematics, state (current/desired) and other characteristics (e.g. contacts, CoM, CoP, limits, actuation).
 */
  template<typename Body>
  LinearPoint(Body body, mwoibn::robot_class::Robot& robot): State(robot.getModel(), robot.state), point(body, robot.getModel(), robot.state){
    _point.setZero(3); // linear point lies in the cartesian 3D space
  }

  using Point::operator=;

  virtual ~LinearPoint() {}

  //! Compute the current point position
  void compute(){_point.noalias() = point.getLinearWorld();}

  //! compute the corresponding Jacobian
  void computeJacobian() {_jacobian.noalias() = point.getPositionJacobian();}

  /** mwoibn::point_handling::Frame is a class that provides a wrapper over the RBDL library to model the reference frame. 
   * The class provides the methods to compute the position, orientation and velocity of a given frame. 
   *
   * An offset from the origin of the RBDL frame to the point can be set with the mwoibn::point_handling::Frame API
   *
   * The Frame class is depraciated; the FramePlus class should be used instead to model the geometry
   * of the frame and the SpatialVelocity (or LinearVelocity/AngularVelocity) class should be used for the corresponding first-order kinematics.
   */
  mwoibn::point_handling::Frame point;


};

} // namespace package
} // namespace library

#endif // __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H
