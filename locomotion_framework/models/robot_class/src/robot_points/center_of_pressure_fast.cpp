#include "mwoibn/robot_points/center_of_pressure_fast.h"

namespace mwoibn
{
namespace robot_points
{

  CenterOfPressureFast::CenterOfPressureFast(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, mwoibn::robot_class::Contacts& contacts)
        : CenterOfPressure(model, state, contacts)
{
    _jacobian.setZero(3, _state.velocity.size());
}

void CenterOfPressureFast::computeJacobian()
{
    _jacobian.setZero();

    for(auto id: _contacts.getActive()){
      _jacobian.noalias() += _forces[id]*_contacts.contact(id).getJacobian(); // this will not update contacts
    }

    _jacobian.noalias() = _jacobian/_sum_force;
}



} // namespace package
} // namespace library
