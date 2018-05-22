#include <mgnss/controllers/steering_v6.h>

mgnss::events::Steering6::Steering6(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::ContactPointTrackingTask& plane,
    mwoibn::VectorN init_pose, double K_icm, double K_sp, double K_v, double dt,
    double margin_icm, double margin_sp, double margin, double max)
  : SteeringReference(robot, plane, init_pose, K_icm, K_sp, dt, margin, max), _K_v(K_v)
{
  _pb_icm.setZero(_size);
  _pb_sp.setZero(_size);

  _damp_sp.setZero(_size);
  _damp_icm.setZero(_size);

  _treshhold_icm = margin_icm/dt;
  _treshhold_sp = margin_sp/dt;
//  std::cout << margin << "\t" << margin_icm << "\t" << margin_sp << std::endl;
//  std::cout << _treshhold << "\t" << _treshhold_icm << "\t" << _treshhold_sp << std::endl;

 // _computeTreshhold();

}

void mgnss::events::Steering6::_resetTreshhold(){
  _treshhold = _treshhold*_dt;

  _treshhold_icm = _treshhold_icm*_dt;
  _treshhold_sp = _treshhold_sp*_dt;
}

void mgnss::events::Steering6::_computeTreshhold(){
  _treshhold = _treshhold/_dt;

  _treshhold_icm = _treshhold_icm/_dt;
  _treshhold_sp = _treshhold_sp/_dt;
//  std::cout << _treshhold << "\t" << _treshhold_icm << "\t" << _treshhold_sp << std::endl;
}


void mgnss::events::Steering6::_merge(int i){
//  if(_resteer[i]) _b_st[i] -= mwoibn::PI; // change velocity sign

  _v[i] = _computeVelocity(i);
  _damp[i] = std::tanh(std::fabs(_v[i]) / _treshhold);
  SteeringReference::_merge(i);

//  std::cout << _heading;

//  std::cout << "\t" << _heading << std::endl;

  _b_st[i] -= _heading;
//  if(i == 1){
//    std::cout << _b[i]*180/mwoibn::PI << ",\t" << _b_st[i]*180/mwoibn::PI;
//  }
  //  _b[i] += 6.28318531 * ( std::floor(_b_st[i] / 6.28318531) -  std::floor(_b[i] / 6.28318531));

  mwoibn::eigen_utils::limitHalfPi(_b_st[i], _b[i]);
  limit2PI(_b_st[i], _b[i]);
//  if(i == 1){
//    std::cout << ",\t" << _b_st[i]*180/mwoibn::PI;
//    std::cout << ",\t" << _damp[i];
//  }
  //_b_st[i] = _b[i];
  _b_st[i] += _damp[i] * (_b[i] - _b_st[i]); // do give the result in the world frame
//  if(i == 1){
//    std::cout << ",\t" << _b_st[i]*180/mwoibn::PI << std::endl;
//  }
  _b_st[i] += _heading;
}




void mgnss::events::Steering6::_ICM(mwoibn::Vector3 next_step)
{

  _pb_icm.noalias() = _b_icm;

  for (int i = 0; i < _size; i++)
  {
    double v_last = _v_icm[i];

    _steerICM(i, next_step);
    _velICM(i);

//    std::cout << "icm " << _b_icm[i] << ", " << _pb_icm[i] << std::endl;
    int factor = mwoibn::eigen_utils::limitHalfPi(_pb_icm[i],_b_icm[i]);
    factor = limit2PI(_pb_icm[i], _b_icm[i], factor);

    _v_icm[i] = std::pow(-1,factor)*_v_icm[i]; // change sign to fit with steering

//    std::cout << "factor " << factor << "-1, " << std::pow(-1,factor) << std::endl;

    _damp_icm[i] = std::tanh(std::fabs(_v_icm[i]) / _treshhold_icm);


    _b_icm[i] = _pb_icm[i] + _damp_icm[i] * (_b_icm[i] - _pb_icm[i]);
    _v_icm[i] = v_last + _damp_icm[i] * (_v_icm[i] - v_last);

  }
}

void mgnss::events::Steering6::_velSP(int i){
  _v_sp[i] = _K_v*_plane_ref.norm();
}

void mgnss::events::Steering6::_PT(int i)
{
  // Desired state

  _pb_sp[i] = _b_sp[i];
  double v_last = _v_sp[i];

  _plane_ref.noalias() = _plane.getReferenceError(i).head(2); // size 2

  _steerSP(i);
  _velSP(i);

  int factor = mwoibn::eigen_utils::limitHalfPi(_pb_sp[i],_b_sp[i]);
  factor = limit2PI(_pb_sp[i], _b_sp[i], factor);

  //limit2PI(_b_sp[i], _pb_sp[i]);
  _v_sp[i] = std::pow(-1,factor)*_v_sp[i]; // change sign to fit with steering

  _damp_sp[i] = std::tanh(std::fabs(_v_sp[i]) / _treshhold_sp);
  _b_sp[i] = _pb_sp[i] + _damp_sp[i] *(_b_sp[i] - _pb_sp[i]);
  _v_sp[i] = v_last + _damp_sp[i] * (_v_sp[i] - v_last);
}
