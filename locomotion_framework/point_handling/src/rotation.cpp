#include "mwoibn/point_handling/rotation.h"

namespace mwoibn
{
namespace point_handling
{

  /** @brief get Position in a world frame
   */
  const Rotation::R&
  Rotation::getWorld(bool update){
    _temp_world = RigidBodyDynamics::CalcBodyWorldOrientation(
                         _model, _state.position.get(), _body_id, update).transpose()*getFixed();

    return _temp_world;
  }

  Rotation::R Rotation::getWorld(bool update) const{
    return RigidBodyDynamics::CalcBodyWorldOrientation(
          _model, _state.position.get(), _body_id, update).transpose()*getFixed();
  }



  // /** @bried returnes full jacobian of a point **/
  // const mwoibn::Matrix&
  // Position::getJacobian(bool update)
  // {
  //
  //   _J.setZero();
  //   CalcPointJacobian6D(_model, _state.position.get(), _body_id, _linear, _J, update);
  //
  //   _J_part.noalias() = _J.topRows<3>();
  //
  //   return _J_part;
  // }
  //
  // mwoibn::Matrix Position::getOrientationJacobian(bool update) const{
  //   mwoibn::Matrix J = mwoibn::Matrix::Zero(_J.rows(), _J.cols());
  //
  //   CalcPointJacobian6D(_model, _state.position.get(), _body_id, _linear, J, update);
  //   mwoibn::Matrix J_part = J.topRows<3>();
  //   return J_part;
  // }

  Rotation::R
  Rotation::getReference(unsigned int reference_id, bool update) const
  {
    return  RigidBodyDynamics::CalcBodyWorldOrientation(
                _model, _state.position.get(), reference_id, update) *
            getWorld();;
  }

  void Rotation::setWorld(const Rotation::R& rotation, bool update)
  {

    _current_fixed = getWorld(update).transpose() * rotation;

  }

  void Rotation::setReference(const Rotation::R& rotation,
                                   unsigned int reference_id,
                                   bool update)
  {
    _current_fixed =
    (RigidBodyDynamics::CalcBodyWorldOrientation(_model, _state.position.get(),
                        reference_id, update) * getWorld()).transpose() * rotation;
  }



} // namespace package
} // namespace library
