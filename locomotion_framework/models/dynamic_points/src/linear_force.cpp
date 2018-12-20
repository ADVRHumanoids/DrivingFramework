#include "mwoibn/dynamic_points/linear_force.h"

namespace mwoibn
{
namespace dynamic_points
{

  void LinearForce::computeJacobian(){
      _jacobian = _contacts_inverse->get();
  }

  void LinearForce::compute(){

    _inertia_inverse->compute(_dynamic_model.getInertia());
    _point_jacobian = _velocity.getJacobian();
    _point_transposed = _point_jacobian.transpose();

    _point_inverse.noalias() = _point_jacobian*_inertia_inverse->get();
    _point_temp.noalias() = _point_inverse*_point_transposed;

    _contacts_inverse->compute(_point_temp);

    _jacobian_temp.noalias() = _contacts_inverse->get()*_point_inverse;
    _point.noalias() = _jacobian*_state[_interface].get();
    //_point.noalias() = _contacts_inverse->get()*_acceleration.getWorld();

  }


} // namespace package
} // namespace library
