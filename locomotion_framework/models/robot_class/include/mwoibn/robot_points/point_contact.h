#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__POINT_CONTACT_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__POINT_CONTACT_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/robot_points/contact_v2.h"

namespace mwoibn
{

namespace robot_points
{

/** @brief This class implements the simple contact characterized by a constant state in its end-point reference frame.
 */
class PointContact : public ContactV2
{

public:
PointContact(RigidBodyDynamics::Model& model,
          const mwoibn::robot_class::State& state,
          YAML::Node config)
        : ContactV2(model, state, config)
{
        _state_size = 3;
        _resize();
        _read(config);
}

PointContact(PointContact&& other)
        : ContactV2(other)
{
        mwoibn::Matrix temp_directions = _directions;
        //_state_size = 6;
        _resize();
        _directions = temp_directions;
}

PointContact(PointContact& other)
        : ContactV2(other)
{
        //_state_size = 6;
        mwoibn::Matrix temp_directions = _directions;
        _resize();
        _directions = temp_directions;
}

virtual ~PointContact() { }

using Point::operator=;

virtual const mwoibn::Matrix& getPointJacobian(mwoibn::Matrix3 rotation_matrix); // contact specific
virtual const mwoibn::Matrix& getWorldJacobian();
virtual const mwoibn::VectorN& getPosition();

const mwoibn::VectorN& getReactionForce(){
  return _wrench.force.getWorld();
}

protected:

virtual void _initDirections(mwoibn::Matrix6 directions);

};
} // namespace package
} // namespace library
#endif // CONTACT_V2_H
