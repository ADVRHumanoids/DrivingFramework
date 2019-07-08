#include "mwoibn/robot_points/center_of_mass.h"

namespace mwoibn
{
namespace robot_points
{

CenterOfMass::CenterOfMass(RigidBodyDynamics::Model& model,
                           const mwoibn::robot_class::State& state)

        : State(model, state), _points(0, model, state)
{

        std::vector<RigidBodyDynamics::Body> bodies = model.mBodies;

        for (int i = 0; i < bodies.size(); i++)
        {
                if (bodies.at(i).mMass)
                {
                        _points.addPoint(bodies.at(i).mCenterOfMass, i, model.GetBodyName(i));
                        _masses.push_back(bodies.at(i).mMass);
                        _mass += bodies.at(i).mMass;
                }
        }
        _points.computeChain();

        _point.setZero(3);

}

void CenterOfMass::compute()
{
  double mass;

  // RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(),
  //                                             mass, _com, nullptr, nullptr, false);
       RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(), nullptr,
                                                  mass, _com, nullptr, nullptr, nullptr, nullptr,false);



        _point.noalias() = _com;
}



void CenterOfMass::computeDerivative()
{
  double mass;

  //RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(),
  //                                            mass, _com, &_d_com, nullptr, false);
   RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(), nullptr,
                                               mass, _com, &_d_com, nullptr, nullptr, nullptr, false);
  _point.noalias() = _com;
}

void CenterOfMass::accelerationComponent(){
  double mass;

  //RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(),
  //                                               mass, _com, &_d_com, nullptr, false);
  RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _state.position.get(), _state.velocity.get(), &_state.zero.get(),
                                              mass, _com, &_d_com, &_dd_com, nullptr, nullptr, false);
  _point.noalias() = _com;
}

void CenterOfMass::computeJacobian()
{

        _jacobian.setZero();

        for (int i = 0; i < _points.size(); i++)
        {
                _jacobian += _masses.at(i) * _points.getPointJacobian(i, false);
        }

        _jacobian.noalias() = _jacobian / _mass;
}



} // namespace package
} // namespace library
