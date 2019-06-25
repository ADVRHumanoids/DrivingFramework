#include "mwoibn/dynamic_points/torus_velocity.h"

namespace mwoibn
{
namespace dynamic_points
{

  void TorusVelocity::computeJacobian(){

      Torus::computeJacobian();
      mwoibn::Matrix3 toN = _torus.groundNormal()*_torus.groundNormal().transpose();
      mwoibn::Matrix3 toPN = mwoibn::Matrix3::Identity() - toN;

     // VELOCITY
     _support_jacobian = _torus.getJacobianWheel()/_robot.rate();

     _constant =    (getDependant()*toPN -_support_jacobian )*_torus.wheelVelocity().angular().getWorld();
     _constant += getIndependant();
     _support_jacobian += getDependant()*toN;
     _support_jacobian = _support_jacobian*_robot.rate();
     _constant = _constant*_robot.rate();

     _jacobian = _support_jacobian * _torus._v_centre.angular().getJacobian(); // under the contact point assumption
     _constant += _torus.getJacobian()*_robot.state.velocity.get();

     // std::cout << "_support_jacobian\t" << _support_jacobian << std::endl;
     // std::cout << "angular\t" <<  _torus._v_centre.angular().getJacobian() << std::endl;
  }

  void TorusVelocity::compute(){
      _point = _torus.get();  //?

  }


} // namespace package
} // namespace library
