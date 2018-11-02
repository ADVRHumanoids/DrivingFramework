#include "mwoibn/point_handling/spatial_velocity.h"

namespace mwoibn
{
namespace point_handling
{

  const mwoibn::Matrix& SpatialVelocity::getJacobian(bool update)  {

    _J.setZero();
    CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), _J, update);
    return _J;
    }


  mwoibn::Matrix SpatialVelocity::getJacobian(bool update) const{
    mwoibn::Matrix J = mwoibn::Matrix::Zero(_J.rows(), _J.cols());

    CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), J, update);

    return J;
  }

  void SpatialVelocity::getJacobian(mwoibn::Matrix& current, bool update) const{
    current.setZero();

    CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), current, update);
    
  }

} // namespace package
} // namespace library
