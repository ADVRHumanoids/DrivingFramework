#include "mwoibn/point_handling/linear_velocity.h"

namespace mwoibn
{
namespace point_handling
{

  const mwoibn::Matrix& LinearVelocity::getJacobian(bool update)  {

      _J.setZero();

      CalcPointJacobian(_model, _state.position.get(), _body_id, frame.position.getFixed(), _J,
                        update);

      return _J;
    }


  mwoibn::Matrix LinearVelocity::getJacobian(bool update) const{
    mwoibn::Matrix J = mwoibn::Matrix::Zero(_J.rows(), _J.cols());

    CalcPointJacobian(_model, _state.position.get(), _body_id, frame.position.getFixed(), J,
                      update);

    return J;
  }


} // namespace package
} // namespace library
