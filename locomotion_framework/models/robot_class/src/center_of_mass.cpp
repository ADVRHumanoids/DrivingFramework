#include "mwoibn/robot_class/center_of_mass.h"

namespace mwoibn
{
namespace robot_class
{

CenterOfMass::CenterOfMass(RigidBodyDynamics::Model& model,
                           const mwoibn::VectorN& positions,
                           const mwoibn::VectorN& velocities)
        : _points(0, model), _model(model), _positions(positions),
        _velocities(velocities)
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

}

void CenterOfMass::compute()
{

        double mass;
        RigidBodyDynamics::Utils::CalcCenterOfMass(_model, _positions, _velocities,
                                                   mass, _com, nullptr, nullptr, false);
}

void CenterOfMass::computeJacobian()
{

        _jacobian = mwoibn::Matrix::Zero(3, _model.dof_count);

//  std::vector<mwoibn::Matrix> temp_jacobians =
//      _points.getFullJacobians(_positions);

        for (int i = 0; i < _points.size(); i++)
        {
                _jacobian += _masses.at(i) * _points.getPointJacobian(i, _positions, false);

//    std::cout << "jacobian i\n" << temp_jacobians.at(i) << std::endl;
        }

        _jacobian = _jacobian / _mass;
}

void CenterOfMass::update(bool jacobian)
{

        compute();
        if (jacobian)
                computeJacobian();
}

} // namespace package
} // namespace library
