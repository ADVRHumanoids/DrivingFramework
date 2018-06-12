#include <mgnss/controllers/steering_v5.h>

mgnss::events::Steering5::Steering5(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::tasks::ContactPointTracking& plane,
    mwoibn::VectorN init_pose, double K_icm, double K_sp, double dt,
    double margin, double max)
  : SteeringReference(robot, plane, init_pose, K_icm, K_sp, dt, margin, max)
{
  _pb_icm.setZero(_size);
  _pb_sp.setZero(_size);

  _damp_sp.setZero(_size);
  _damp_icm.setZero(_size);

  _treshhold = _treshhold/_dt;
//  _computeTreshhold();

}

void mgnss::events::Steering5::_computeTreshhold(){
  _treshhold = _treshhold/_dt;
}

void mgnss::events::Steering5::_resetTreshhold(){
  _treshhold = _treshhold*_dt;
}

void mgnss::events::Steering5::_merge(int i){
  if(_resteer[i]) _b_st[i] -= mwoibn::PI; // change velocity sign

  double vel = _computeVelocity(i);

  SteeringReference::_merge(i);

  _b[i] += _heading;

  _b[i] += 6.28318531 * ( std::floor(_b_st[i] / 6.28318531) -  std::floor(_b[i] / 6.28318531));

  limit2PI(_b_st[i], _b[i]);

  _b_st[i] += std::tanh(std::fabs(vel) / _treshhold) * (_b[i] - _b_st[i]); // do give the result in the world frame

}

void mgnss::events::Steering5::_ICM(mwoibn::Vector3 next_step)
{

  _pb_icm.noalias() = _b_icm;

  for (int i = 0; i < _size; i++)
  {
    _steerICM(i, next_step);
    mwoibn::eigen_utils::limitHalfPi(_b_icm[i], _pb_icm[i]);
    limit2PI(_b_icm[i], _pb_icm[i]);

    double v_last = _v_icm[i];

    _velICM(i);

    _damp_icm[i] = std::tanh(std::fabs(_v_icm[i]) / _treshhold);


    _b_icm[i] = _pb_icm[i] + _damp_icm[i] * (_b_icm[i] - _pb_icm[i]);
    _v_icm[i] = v_last + _damp_icm[i] * (_v_icm[i] - v_last);

  }
}

void mgnss::events::Steering5::_PT(int i)
{
  // Desired state

  _pb_sp[i] = _b_sp[i];

  _plane_ref.noalias() = _plane.getReferenceError(i).head(2); // size 2

  _steerSP(i);
  double v_last = _v_sp[i];

  mwoibn::eigen_utils::limitHalfPi(_b_sp[i], _pb_sp[i]);
  limit2PI(_b_sp[i], _pb_sp[i]);

  _velSP(i);

  _damp_sp[i] = std::tanh(std::fabs(_v_sp[i]) / _treshhold);
  _b_sp[i] = _pb_sp[i] + _damp_sp[i] *(_b_sp[i] - _pb_sp[i]);
  _v_sp[i] = v_last + _damp_sp[i] * (_v_sp[i] - v_last);
}
