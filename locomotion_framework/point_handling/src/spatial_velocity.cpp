#include "mwoibn/point_handling/spatial_velocity.h"

namespace mwoibn
{
namespace point_handling
{

  const mwoibn::Matrix& SpatialVelocity::getJacobian(bool update)  {

    _J.setZero();
    CalcPointJacobian6D(_model, _state.get(), _body_id, _position.getFixed(), _J, update);
    return _J;
    }


  mwoibn::Matrix SpatialVelocity::getJacobian(bool update) const{
    mwoibn::Matrix J = mwoibn::Matrix::Zero(_J.rows(), _J.cols());

    CalcPointJacobian6D(_model, _state.get(), _body_id, _position.getFixed(), J, update);

    return J;
  }



} // namespace package
} // namespace library
