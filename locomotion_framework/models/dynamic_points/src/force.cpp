#include "mwoibn/dynamic_points/force.h"

namespace mwoibn
{
namespace dynamic_points
{

  void Force::computeJacobian(){
    _jacobian = _contacts_inverse->get();
  }

  void Force::compute(){

    //_inertia_inverse->compute(_dynamic_model.getInertia());
    _point_jacobian = _frame.getJacobian();
    _point_transposed = _point_jacobian.transpose();

    _point_inverse.noalias() = _point_jacobian*_dynamic_model.getInertiaInverse();
    _point_temp.noalias() = _point_inverse*_point_transposed;

    _contacts_inverse->compute(_point_temp);

    _jacobian_temp.noalias() = _contacts_inverse->get()*_point_inverse;
    _point.noalias() = _jacobian_temp*(_state[_interface].get());
    _point -= _point_inverse*_frame.getConstant();

  }


} // namespace package
} // namespace library
