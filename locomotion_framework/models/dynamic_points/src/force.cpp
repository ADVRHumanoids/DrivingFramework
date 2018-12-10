#include "mwoibn/dynamic_points/force.h"

namespace mwoibn
{
namespace dynamic_points
{

  void Force::computeJacobian(){
      // _jacobian = _contacts_inverse->get();
  }

  void Force::compute(){

    _inertia_inverse->compute(_dynamic_model.getInertia());
    _temp = _frame.getJacobian();
    _contacts_inverse->compute(_temp*_inertia_inverse->get()*_temp.transpose()); // NOT RT SAVE - this should be separated
    mwoibn::Vector3 force = _temp*_inertia_inverse->get()*_state[_interface].get();
    _jacobian = _contacts_inverse->get()*_temp*_inertia_inverse->get();
    _point = _contacts_inverse->get()*force;

  }


} // namespace package
} // namespace library
