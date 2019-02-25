#include "mwoibn/dynamic_points/torus.h"

namespace mwoibn
{
namespace dynamic_points
{

  void Torus::computeJacobian(){
    // std::cout << "velocity\t" << _state.velocity.get().transpose() << std::endl;
    // std::cout << "acceleration\t" << _state.acceleration.get().transpose() << std::endl;

      mwoibn::Vector3 alf = _torus._axis_world*_torus._ground_normal.transpose()*_torus._axis_world;
      double ksi = 1/std::sqrt(1 - alf.transpose()*_torus._ground_normal );
      mwoibn::Vector3 w_wheel = _torus._v_centre.angular().getWorld();

      mwoibn::Matrix3 s_alf, eta, s_wheel, s_y;
      mwoibn::Matrix3 I = mwoibn::Matrix3::Identity();

      mwoibn::eigen_utils::skew(alf, s_alf);
      mwoibn::eigen_utils::skew(w_wheel, s_wheel);
      mwoibn::eigen_utils::skew(_torus._axis_world, s_y);


      eta = _torus._axis_world*_torus._ground_normal.transpose()*s_y;
      mwoibn::Matrix3 zeta = 0.5*ksi*ksi*(_torus._ground_normal - alf)*_torus._ground_normal.transpose();
      mwoibn::Vector3 eta_w = eta*w_wheel;
      double k = _torus._ground_normal.transpose()*s_alf*w_wheel;
      mwoibn::Matrix3 s_alf_k;
      mwoibn::Vector3 alf_k = alf*k;

      mwoibn::eigen_utils::skew(alf_k, s_alf_k);

      mwoibn::Matrix3 s_n;
      mwoibn::eigen_utils::skew(_torus._ground_normal, s_n);
      mwoibn::Vector3 a_w = s_alf*w_wheel;

      mwoibn::Matrix3 s_eta_w, s_a_w;
      mwoibn::eigen_utils::skew(eta_w, s_eta_w);
      mwoibn::eigen_utils::skew(a_w, s_a_w);

      // std::cout << "eta_w\t" << eta_w.transpose() << std::endl;
      // std::cout << "s_eta_w\t" << s_eta_w << std::endl;

      // mwoibn::Matrix3 toN = _torus._ground_normal*_torus._ground_normal.transpose();
      // mwoibn::Matrix3 toPN = I - _torus._ground_normal*_torus._ground_normal.transpose();
      // double Nk = _torus._ground_normal.transpose()*s_alf*toN*w_wheel;
      // mwoibn::Matrix3 s_Nwheel;
      // mwoibn::eigen_utils::skew(toN*w_wheel, s_Nwheel);

      // std::cout << "1\t" << (ksi*ksi*ksi*_torus._R*(2*eta*w_wheel)*Nk).norm() << std::endl;
      // std::cout << "2\t" << (ksi*ksi*ksi*_torus._R*(-mwoibn::Matrix3::Constant(3*ksi*ksi*k)*_torus._ground_normal)*Nk).norm() << std::endl;
      // std::cout << "3\t" << (ksi*ksi*ksi*_torus._R*(mwoibn::Matrix3::Constant(3*ksi*ksi*k)*alf)*Nk).norm() << std::endl;
      // std::cout << "4\t" << (2*ksi*_torus._R*zeta*s_wheel*eta*toN*w_wheel).norm() << std::endl;
      // std::cout << "_1\t" << (ksi*ksi*ksi*_torus._R*(2*eta*toN*w_wheel)*k).norm() << std::endl;
      // std::cout << "_2\t" << (ksi*ksi*ksi*_torus._R*(-mwoibn::Matrix3::Constant(3*ksi*ksi*Nk)*_torus._ground_normal)*k).norm() << std::endl;
      // std::cout << "_3\t" << (ksi*ksi*ksi*_torus._R*(mwoibn::Matrix3::Constant(3*ksi*ksi*Nk)*alf)*k).norm() << std::endl;
      // std::cout << "_4\t" << (2*ksi*_torus._R*zeta*s_Nwheel*eta*w_wheel).norm() << std::endl;


      // mwoibn::Vector3 alf_Nk = alf*Nk;
      // mwoibn::Matrix3 s_alf_Nk;
      // mwoibn::eigen_utils::skew(alf_Nk, s_alf_Nk);
      //
      // mwoibn::Vector3 eta_Nw = eta*toN*w_wheel;
      // mwoibn::Matrix3 s_eta_Nw;
      // mwoibn::eigen_utils::skew(eta_Nw, s_eta_Nw);

      // std::cout << "d1\t" << (zeta*s_wheel*s_alf*toN*w_wheel).norm() << std::endl;
      // std::cout << "d1.5\t" << (- I*s_wheel*s_alf*toN*w_wheel).norm() << std::endl; // this is wrong
      // std::cout << "_d1.5\t" << (- I*s_Nwheel*s_alf*w_wheel).norm() << std::endl; // this is wrong

      // std::cout << "d2\t" << ((zeta - I)*_torus._axis_world*_torus._ground_normal.transpose()*s_wheel*s_y*toN*w_wheel).norm() << std::endl;
      // std::cout << "d3\t" << (2* s_eta_w*toN*w_wheel).norm() << std::endl;
      // std::cout << "d4\t" << (2*ksi*ksi*s_alf_k*toN*w_wheel).norm() << std::endl;



      _dependend = ksi*_torus._R*((zeta - I)*s_wheel*s_alf + (zeta - I)*_torus._axis_world*_torus._ground_normal.transpose()*s_wheel*s_y + 2* s_eta_w + 2*ksi*ksi*s_alf_k);
      // _dependend = ksi*_torus._R*(zeta*s_wheel*s_alf + (zeta - I)*_torus._axis_world*_torus._ground_normal.transpose()*s_wheel*s_y + 2* s_eta_w + 2*ksi*ksi*s_alf_k);

      _independend = ksi*ksi*ksi*_torus._R*(2*eta*w_wheel -mwoibn::Matrix3::Constant(3*ksi*ksi*k)*(_torus._ground_normal - alf))*k + 2*ksi*_torus._R*zeta*s_wheel*eta*w_wheel;

      _constant = _independend + _dependend*w_wheel;

      mwoibn::Matrix j_test__ = (zeta*(eta+s_alf)-s_n-eta  )*ksi*_torus._R - s_n*_torus._r;

      // std::cout << "_constant\n" << _constant << std::endl;
      // std::cout << "angular().getWorld\n" << _torus._v_centre.angular().getWorld().transpose() << std::endl;

      est_ = last_ + (_constant + j_test__*_torus._v_centre.angular().getJacobian()*_state.acceleration.get())*0.005;

      last_ = _torus.getJacobian()*_state.velocity.get();

  }

  void Torus::compute(){
      //acceleration or force?
  }


} // namespace package
} // namespace library
