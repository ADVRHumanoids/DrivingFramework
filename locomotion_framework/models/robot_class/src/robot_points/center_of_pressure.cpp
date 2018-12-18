#include "mwoibn/robot_points/center_of_pressure.h"

namespace mwoibn
{
namespace robot_points
{

  CenterOfPressure::CenterOfPressure(RigidBodyDynamics::Model& model,
               const mwoibn::robot_class::State& state, mwoibn::robot_class::Contacts& contacts, mwoibn::robot_points::CenterOfMass& com)
        : State(model, state), _contacts(contacts), _points("ROOT", _model, _state), _com(com)
{
}

void CenterOfPressure::init(){
  _null_space.assign(_contacts.size(), mwoibn::Matrix3::Zero());

  _points.clear();

  for(int i = 0; i < _contacts.size(); i++)
  _points.addPoint(_contacts.contact(i).wrench().getBodyId());

  _point.setZero(3);
  _forces.setZero(_contacts.size());
}

void CenterOfPressure::compute()
{
    _sum_force = 0;
    _forces.setZero();
    _point.setZero();

    int id = 0;
    for(auto& contact: _contacts){
        if (!contact->isActive()) continue;

        mwoibn::Vector3 n = contact->getGroundNormal();

        _forces[id] = n.dot(contact->wrench().force.getWorld());
        _sum_force += _forces[id];
        _null_space[id] = n*n.transpose();
        _null_space[id] = mwoibn::Matrix3::Identity() - _null_space[id];
        _temp = _null_space[id]*contact->get().head<3>();
        _point += _temp*_forces[id];
        id++;
        //_contacts.contact(id).getPosition()
    }

    _point = _point/_sum_force;
}

void CenterOfPressure::update(bool jacobian){

    for(int i = 0; i < _contacts.size(); i++)
        _contacts.contact(i).update(jacobian);

    Point::update(jacobian);
    compute();
}



} // namespace package
} // namespace library
