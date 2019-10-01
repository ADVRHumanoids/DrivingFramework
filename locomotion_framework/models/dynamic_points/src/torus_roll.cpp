#include "mwoibn/dynamic_points/torus_roll.h"

namespace mwoibn
{
namespace dynamic_points
{

  void TorusRoll::computeJacobian(){
    mwoibn::Matrix3 s_cp;

    mwoibn::eigen_utils::skew(_torus.positionOffset(), s_cp);
    // std::cout << "s_cp\n" << s_cp << std::endl;
    _wheel_jacobian = _torus.getJacobianWheel();

    _wheel_jacobian += s_cp;
    _jacobian.noalias() = _wheel_jacobian*_torus._v_centre.angular().getJacobian();
  }

  void TorusRoll::compute(){
    mwoibn::Matrix3 mat_1_, mat_2_;
    mwoibn::Vector3 vec_1_, vec_2_, vec_3_;
    double s_1_;


    // wheel angular velocity
    mwoibn::Vector3 wheel_ = _torus._angular_world;
    //mwoibn::Vector3 n_wheel_ = _torus._ground_normal*_torus._ground_normal.transpose()*_torus._angular_world;
    double n_y_ =  _torus._ground_normal.transpose()*_torus._axis_world;
    mwoibn::Vector3 alf_ = _torus._axis_world*n_y_;
    // auxiliary variables

    double n_alf_ = alf_.transpose()*_torus._ground_normal;
    double ksi = 1/std::sqrt(1 - n_alf_ );

    // skew matrices
    mwoibn::Matrix3 s_alf_, s_y_, s_wheel_, s_gamma_;
    mwoibn::eigen_utils::skew(alf_, s_alf_);
    mwoibn::Vector3 alf_omega_ = s_alf_*wheel_;

    mwoibn::eigen_utils::skew(_torus._axis_world, s_y_);
    mwoibn::eigen_utils::skew(wheel_, s_wheel_);
    //mwoibn::eigen_utils::skew(n_wheel_, s_n_wheel_);

    vec_1_.noalias() = _torus._ground_normal.transpose()*s_y_;
    mat_1_.noalias() = _torus._axis_world*vec_1_.transpose();
    mwoibn::Matrix3 gamma_ = mat_1_ + s_alf_;
    mwoibn::Vector3 gamma_omega_ = gamma_*wheel_;
    mwoibn::eigen_utils::skew(gamma_omega_, s_gamma_);


    vec_1_.noalias() = _torus._ground_normal.transpose()*s_wheel_;
    mwoibn::Matrix3 eta_ = _torus._axis_world*vec_1_.transpose();

    vec_1_ =  _torus._ground_normal - alf_;
    mat_1_.noalias() = vec_1_*_torus._ground_normal.transpose();
    mwoibn::Matrix3 zeta_ = 0.5*ksi*ksi*mat_1_;

    // compute the offset
    mat_1_ = -3*zeta_ + 2*mwoibn::Matrix3::Identity();
    s_1_ = _torus._ground_normal.transpose()*alf_omega_;
    mat_2_.noalias() = mat_1_*gamma_;
    _velocity_jacobian = ksi*ksi*mat_2_*s_1_;
    // std::cout << "1\n" << _velocity_jacobian << std::endl;
    mat_1_.noalias() = zeta_*s_wheel_;
    mat_1_ = 2*mat_1_;
    _velocity_jacobian.noalias() += mat_1_*gamma_;

    // std::cout << "2 alf\n" << mat_1_*s_alf_ << std::endl;
    // std::cout << "2 eta\n" << mat_1_*_torus._axis_world*_torus._ground_normal.transpose()*s_y_ << std::endl;
    vec_1_.noalias() = zeta_*gamma_omega_;
    mwoibn::eigen_utils::skew(vec_1_, mat_1_);
    _velocity_jacobian += mat_1_;
    // std::cout << "3\n" << mat_1_ << std::endl;

    vec_1_.noalias() = eta_*_torus._axis_world;
    mwoibn::eigen_utils::skew(vec_1_, mat_1_);
    _velocity_jacobian -= mat_1_;
    // std::cout << "4\n" << -mat_1_ << std::endl;

    mat_1_.noalias() = eta_*s_y_;
    _velocity_jacobian -= mat_1_;
    _velocity_jacobian = ksi*_torus._R*_velocity_jacobian;
    _constant.noalias() = _velocity_jacobian*wheel_;
    // std::cout << "5\n" << -mat_1_ << std::endl;

  }


} // namespace package
} // namespace library
