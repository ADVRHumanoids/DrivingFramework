#ifndef __MWOIBN__HIERARCHICAL_CONTROL__TASKS__ADMITANCE_COM_H
#define __MWOIBN__HIERARCHICAL_CONTROL__TASKS__ADMITANCE_COM_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
//#include <rbdl/rbdl.h>
//#include "mwoibn/point_handling/robot_points_handler.h"
#include "mwoibn/point_handling/robot_points_handler.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 **to control the position of a point defined in one of a robot reference frames
 *
 */
class AdmitanceCom : public BasicTask
{

public:
/**
 * in this version it is equivlent to standard position control
 */
AdmitanceCom(mwoibn::robot_class::CenterOfMass& com, double Kp)
        : BasicTask(), _com(com), _Kp(Kp)
{
        _init(2, _robot.getDofs());
        //_reference = 2;
        _f_des.setZero(2);
        _f.setZero(2);
        _m_des.setZero(2);
}

virtual ~AdmitanceCom() {
}

//! updates task error based on the current state of the robot and task
// reference position
virtual void setReferene(double x, double y){
  _m_des[0] = x;
  _m_des[1] = y;
}

virtual mwoibn::VectorN& getReferene(){
  return _m_des;
}

virtual void setReferene(int i, double x){
  _m_des[i] = x;
}

virtual double getReferene(int i){
  return _m_des[i];
}

virtual void updateError(){

  // _f_des = _robot.centerOfMass().mass * (_a + _Kp*(_m_des - _robot.centerOfMass().get()));
  // _f = _robot.centerOfMass().mass * _a;
  // e = _f_des - _f;

  // In this version the acceleration cancels out, Kp simplifies to a task gain
  _error = _com.mass * (_m_des - _com.get()));
  // F_des = m*ddot m_des
  // F_com = force - with
  //          ile to jest JH^-1JT dla CoM w porównaniu to M_com
  //          (JH^-1JT)-1 dd-m = J H^-1 (\tau - C - F_g - '\dot J \dot q' - F_const)
}
//! updates task Jacobian based on the current state of the robot
virtual void updateJacobian(){
  _jacobian.noalias() = _com.mass()*_com.getJacobian().topRows<2>();
}
//! sets task reference

protected:
  //mwoibn::robot_class::Robot& _robot;
  mwoibn::robot_class::CenterOfMass& _com;
  mwoibn::VectorN _f_des, _f, _m_des;
  double _Kp;
};
}
} // namespace package
} // namespace library
#endif
