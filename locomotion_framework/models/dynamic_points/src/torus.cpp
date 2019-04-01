#include "mwoibn/dynamic_points/torus.h"

namespace mwoibn
{
namespace dynamic_points
{

  void Torus::computeJacobian(){

      double scl = _torus._ground_normal.transpose()*_torus._axis_world;
      mwoibn::Vector3 alf = _torus._axis_world*scl;

      double scl2 = alf.transpose()*_torus._ground_normal;
      double ksi = 1/std::sqrt(1 - scl2 );

      mwoibn::Vector3 w_wheel = _torus._v_centre.angular().getWorld();

      mwoibn::Matrix3 s_alf, eta, s_wheel, s_y;
      mwoibn::Matrix3 I = mwoibn::Matrix3::Identity();

      mwoibn::eigen_utils::skew(alf, s_alf);
      mwoibn::eigen_utils::skew(w_wheel, s_wheel);
      mwoibn::eigen_utils::skew(_torus._axis_world, s_y);

      mwoibn::Vector3 vec_1, vec_2, vec_3, eta_w;
      mwoibn::Matrix3 zeta, mat_1;

      mat_1.noalias() = _torus._axis_world*_torus._ground_normal.transpose();
      eta.noalias() = mat_1*s_y;

      vec_1 = _torus._ground_normal - alf;
      zeta.noalias() = 0.5*ksi*ksi*vec_1*_torus._ground_normal.transpose();

      eta_w.noalias() = eta*w_wheel;
      vec_3.noalias() = s_alf*w_wheel;

      double k = _torus._ground_normal.transpose()*vec_3;

      mwoibn::Matrix3 s_alf_k;
      mwoibn::Vector3 alf_k;
      alf_k.noalias() = alf*k;

      mwoibn::eigen_utils::skew(alf_k, s_alf_k);

      mwoibn::Matrix3 s_n;
      mwoibn::eigen_utils::skew(_torus._ground_normal, s_n);

      mwoibn::Matrix3 s_eta_w;
      mwoibn::eigen_utils::skew(eta_w, s_eta_w);


      _dependend = 2*ksi*ksi*s_alf_k;
      _dependend += 2* s_eta_w;

      mwoibn::Matrix3 mat_2, mat_3, mat_4, mat_5;
      mat_2 = zeta - I;
      mat_3.noalias() = s_wheel*s_y;
      mat_4.noalias() = mat_1*mat_3;
      mat_5.noalias() = s_wheel*s_alf;

      mat_4 += mat_5;
      mat_5.noalias() =  mat_2*mat_4;
      _dependend += mat_5;
      _dependend =  ksi*_torus._R*_dependend;

      // _dependend = ksi*_torus._R*((zeta - I)*s_wheel*s_alf + (zeta - I)*mat_1*s_wheel*s_y + 2* s_eta_w + 2*ksi*ksi*s_alf_k);
      vec_2.noalias() = 2*eta*w_wheel;
      mat_2 = mwoibn::Matrix3::Constant(3*ksi*ksi*k);
      vec_3.noalias() = mat_2*vec_1;
      vec_2 -= vec_3;
      _independend = ksi*ksi*ksi*_torus._R*vec_2*k;
      vec_2.noalias() = eta*w_wheel;
      vec_3.noalias() = s_wheel*vec_2;
      vec_2.noalias() = zeta*vec_3;
      _independend += 2*ksi*_torus._R*vec_2;


      // _independend = ksi*ksi*ksi*_torus._R*(2*eta*w_wheel -mwoibn::Matrix3::Constant(3*ksi*ksi*k)*vec_1)*k + 2*ksi*_torus._R*zeta*s_wheel*eta*w_wheel;

      _constant = _independend;
      _constant.noalias() += _dependend*w_wheel;

      mat_2 = eta+s_alf;
      mat_3.noalias() = zeta*mat_2;
      mat_3 -= s_n;
      mat_3 -= eta;
      mwoibn::Matrix3 j_test__;
      j_test__.noalias() = mat_3*ksi*_torus._R;
      j_test__ -= s_n*_torus._r;

      vec_1.noalias() = _torus._v_centre.angular().getJacobian()*_state.acceleration.get();
      est_.noalias() = j_test__*vec_1;
      est_ += _constant;
      est_ = est_*0.005; // I need robot rate here
      est_ += last_;
      // est_ = last_ + (_constant + j_test__*_torus._v_centre.angular().getJacobian()*_state.acceleration.get())*0.005;

      last_.noalias() = _torus.getJacobian()*_state.velocity.get();

  }

  void Torus::compute(){
      //acceleration or force?
  }


} // namespace package
} // namespace library
