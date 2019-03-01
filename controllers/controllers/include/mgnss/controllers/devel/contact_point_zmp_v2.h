#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_ZMP_V2_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_ZMP_V2_H

#include "mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h"
#include "mwoibn/robot_points/point.h"
#include "mwoibn/robot_points/ground_wheel.h"
#include "mwoibn/robot_points/torus_model.h"



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
class ContactPointZMPV2 : public ContactPoint3DRbdl
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPointZMPV2(std::vector<std::string> names, mwoibn::robot_class::Robot& robot, YAML::Node config, mwoibn::robot_points::Point& base_point, std::string base_link, double gain)
      : ContactPoint3DRbdl(names, robot, config, base_point, base_link), _gain(gain)
  {  _tracking = true;}

  double forceFactor() {return 1-_force_factor;}
  double comFactor() {return _force_factor;}
  void tracking() { _tracking = true; }
  void balance(){ _tracking = false; _error.setZero();}

protected:
  double _force_factor, _gain;
  bool _tracking;


  virtual void _updateError(){
    // if(_tracking) 
    ContactPoint3DRbdl::_updateError();
    // else {  _error.setZero(); _full_error.setZero();}


    // std::cout << "_tracking\n" << _tracking << std::endl;
  }

  virtual void _updateState(){
      //
      _robot.centerOfMass().update(true);
      _base_point.update(true);

      //
       _force_factor = _robot.gravity().transpose()*_ground_normal;
       _force_factor = 1 / _force_factor;
       _force_factor = _gain/_robot.centerOfMass().mass()*_robot.centerOfMass().get()[2]*_force_factor;
       _force_factor = 1 - _force_factor;

       //std::cout << "_force_factor  " << _force_factor << std::endl;
      _base_point.multiplyJacobian(_force_factor);

      ContactPointTracking::_updateState();
  }





};
}
} // namespace package
} // namespace library
#endif
