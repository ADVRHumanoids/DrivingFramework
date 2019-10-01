#include "mwoibn/dynamic_points/torus2.h"

namespace mwoibn
{
namespace dynamic_points
{

  void Torus2::computeJacobian(){
    _wheel_jacobian = _torus.getJacobianWheel();
    _jacobian = _wheel_jacobian*_torus._v_centre.angular().getJacobian();
  }

  void Torus2::compute(){
    mwoibn::Matrix3 mat_1_, mat_2_;
    mwoibn::Vector3 vec_1_, vec_2_, vec_3_;

    // wheel angular velocity
    mwoibn::Vector3 wheel_ = _torus._angular_world;
    double n_y_ =  _torus._ground_normal.transpose()*_torus._axis_world;
    mwoibn::Vector3 alf_ = _torus._axis_world*n_y_;
    // auxiliary variables

    double n_alf_ = alf_.transpose()*_torus._ground_normal;
    double ksi = 1/std::sqrt(1 - n_alf_ );

    // skew matrices
    mwoibn::Matrix3 s_alf_, s_y_, s_wheel_;
    mwoibn::eigen_utils::skew(alf_, s_alf_);
    mwoibn::eigen_utils::skew(_torus._axis_world, s_y_);
    mwoibn::eigen_utils::skew(wheel_, s_wheel_);

    mwoibn::Vector3 alf_omega_ = s_alf_*wheel_;


    vec_1_ = _torus._ground_normal.transpose()*s_y_;
    mat_1_ = _torus._axis_world*vec_1_.transpose();
    mwoibn::Matrix3 gamma_ = mat_1_ + s_alf_;
    mwoibn::Vector3 gamma_omega_ = gamma_*wheel_;

    vec_1_ = _torus._ground_normal.transpose()*s_wheel_;
    mwoibn::Matrix3 eta_ = _torus._axis_world*vec_1_.transpose();

    vec_1_ =  _torus._ground_normal - alf_;
    mat_1_ = vec_1_*_torus._ground_normal.transpose();
    mwoibn::Matrix3 zeta_ = 0.5*ksi*ksi*mat_1_;


    _constant = ksi*ksi*n_alf_*gamma_omega_*ksi*_torus._R;
    mat_1_ = zeta_ - mwoibn::Matrix3::Identity();

    mat_1_ = 2*mat_1_;
    vec_1_ = s_wheel_*gamma_omega_;
    _constant += mat_1_*vec_1_*ksi*_torus._R;
    _constant += s_wheel_*alf_omega_*ksi*_torus._R;
    vec_1_ = s_wheel_*_torus._axis_world;
    _constant += eta_*vec_1_*ksi*_torus._R;
    _constant -= 3*ksi*ksi*n_alf_*_torus.getJacobianWheel()*wheel_;

    _point = _jacobian*_state.acceleration.get() + _constant;


  }


} // namespace package
} // namespace library
