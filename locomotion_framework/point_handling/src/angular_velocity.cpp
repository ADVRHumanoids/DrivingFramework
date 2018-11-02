#include "mwoibn/point_handling/angular_velocity.h"

namespace mwoibn
{
namespace point_handling
{

  const mwoibn::Matrix& AngularVelocity::getJacobian(bool update)  {


      _J_full.setZero();
      CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), _J_full, update);

      _J.noalias() = _J_full.topRows<3>();
      
      return _J;
    }


  mwoibn::Matrix AngularVelocity::getJacobian(bool update) const{
    mwoibn::Matrix J_full = mwoibn::Matrix::Zero(_J_full.rows(), _J_full.cols());

    CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), J_full, update);
    mwoibn::Matrix J = J_full.topRows<3>();

    return J;
  }

  void AngularVelocity::getJacobian(mwoibn::Matrix& current, bool update) const{
    current.setZero();

    CalcPointJacobian6D(_model, _state.position.get(), _body_id, frame.position.getFixed(), current, update);

    current.bottomRows<3>().setZero();

  }
} // namespace package
} // namespace library
