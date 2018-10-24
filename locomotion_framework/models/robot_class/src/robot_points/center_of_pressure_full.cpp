#include "mwoibn/robot_class/center_of_pressure_full.h"

namespace mwoibn
{
namespace robot_points
{

  CenterOfPressureFull::CenterOfPressureFull(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, mwoibn::robot_class::Contacts contacts)
        : CenterOfPressureFixed(model, state, contacts)
{
    _jacobian.setZero(3, _state.size());
}

void CenterOfPressureFull::computeJacobian()
{
    _jacobian.setZero();


    for(auto id: _contacts.getActive()){
      _jacobian.noalias() += _forces[id]*_contacts.contact(id).getJacobian(); // this will not update contacts
    }

    _jacobian.noalias() = _jacobian/_sum_force;
}



} // namespace package
} // namespace library
