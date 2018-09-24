#include "mgnss/higher_level/steering_reactif.h"

mgnss::higher_level::SteeringReactif::SteeringReactif(
        mwoibn::robot_class::Robot& robot,
        mwoibn::hierarchical_control::tasks::ContactPointTracking& plane,
        const mwoibn::VectorN& contact_vel,
        mwoibn::VectorN init_pose, double K_icm, double K_sp, double K_v, double dt,
        double margin_icm, double margin_sp, double margin, double max)
        : SteeringReference(robot, plane, init_pose, K_icm, K_sp, dt, margin, max), _K_v(K_v), _contact_vel(contact_vel)
{
        _pb_icm.setZero(_size);
        _pb_sp.setZero(_size);
        _pb.setZero(_size);

        _damp_sp.setZero(_size);
        _damp_icm.setZero(_size);

        _reactif_gain.setZero(_size);

        _treshhold_icm = margin_icm/dt;
        _treshhold_sp = margin_sp/dt;

        _last_state = _plane.getState(); // will it initialize correctly?

}

void mgnss::higher_level::SteeringReactif::_resetTreshhold(){
        _treshhold = _treshhold*_dt;

        _treshhold_icm = _treshhold_icm*_dt;
        _treshhold_sp = _treshhold_sp*_dt;
}

void mgnss::higher_level::SteeringReactif::_computeTreshhold(){
        _treshhold = _treshhold/_dt;

        _treshhold_icm = _treshhold_icm/_dt;
        _treshhold_sp = _treshhold_sp/_dt;
}


void mgnss::higher_level::SteeringReactif::_merge(int i){
        if(_resteer[i]) {
                _b[i] -= mwoibn::PI;
                mwoibn::eigen_utils::wrapToPi(_b[i]);
        } // change velocity sign

        _pb[i] = _b[i];

        _v[i] = _computeVelocity(i);
        _damp[i] = std::tanh(std::fabs(_v[i]*_reactif_gain[i]/_dt) /_treshhold/ _treshhold_icm);
        _reactifGain(i);
        _b[i] = std::atan2(  _K_icm * _v_icm[i]  * std::sin(_b_icm[i]) +
                             _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::sin(_b_sp[i]),
                             _K_icm * _v_icm[i]  * std::cos(_b_icm[i]) +
                             _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::cos(_b_sp[i]));
                             std::cout << std::fixed;
                             std::cout << std::setprecision(8);
        //std::cout << i << "\t_K_icm: " << _K_icm << "\t_K_sp: " << _K_sp << "\t_v_icm: " << _v_icm[i] << "\t_v_sp: " << _v_sp[i] <<  "\t_b_icm: " << _b_icm[i] << "\t_b_sp: " << _b_sp[i] << "\trg: " << _reactif_gain[i] << "\t_K_sp|_r_gain|_v_sp" << _K_sp  * std::fabs(_reactif_gain[i]) * _v_sp[i] << std::endl;
        std::cout << i << "\t"
                       << _b[i];
//                       << "\t"
//                       << _K_icm * _v_icm[i]
//                       << "\t"
//                       << _K_icm * _v_icm[i]  * std::sin(_b_icm[i])
//                       << "\t"
//                       << _K_icm * _v_icm[i]  * std::cos(_b_icm[i])
//                       << "\t"
//                       << _K_sp  * std::fabs(_reactif_gain[i]) * _v_sp[i]/_dt
//                       << "\t"
//                       << _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::sin(_b_sp[i])
//                       << "\t"
//                       << _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::cos(_b_sp[i])
//                       << std::endl;

        _raw[i] = _b[i];
        _limited[i] = _b[i];

        limit2PI(_pb[i], _limited[i]);

        _b[i] = _pb[i] + _damp[i] * (_limited[i] - _pb[i]); // do give the result in the world frame
                       std::cout << "\t" << _b[i] << "\t" << _damp[i] << std::endl;

        _b_st[i] = _b[i] + _heading;

}

void mgnss::higher_level::SteeringReactif::_steerICM(int i, const mwoibn::Vector3 next_step){

        SteeringReference::_steerICM(i, next_step);

        _x += _contact_vel[3*i];
        _y += _contact_vel[3*i+1];
        _b_icm[i] = std::atan2(_y, _x); // this uses atan2 to avoid
                                        // singularities in a atan
                                        // implementation
}

void mgnss::higher_level::SteeringReactif::_reactifGain(int i){

        double z = (_plane.getState()[2] - _last_state[2]);
        mwoibn::eigen_utils::wrapToPi(z);

        _plane_ref.noalias() = _plane.getReferenceError(i).head(2);

        double x = std::cos(_heading) * (_plane.getState()[0] - _last_state[0]);
        x += std::sin(_heading) * (_plane.getState()[1] - _last_state[1]);
        x -= _plane_ref[1] * z;

        double y = -std::sin(_heading) * (_plane.getState()[0] - _last_state[0]);
        y += std::cos(_heading) * (_plane.getState()[1] - _last_state[1]);
        y += _plane_ref[0] * z;

        double b = std::atan2(y, x);

        _reactif_gain[i] = x * std::cos(b);
        _reactif_gain[i] += y * std::sin(b);
}

void mgnss::higher_level::SteeringReactif::_ICM(mwoibn::Vector3 next_step)
{

        _pb_icm.noalias() = _b_icm;

        for (int i = 0; i < _size; i++)
        {
                double v_last = _v_icm[i];

                _steerICM(i, next_step);
                _velICM(i);

                int factor = mwoibn::eigen_utils::limitHalfPi(_pb_icm[i],_b_icm[i]);
                factor = limit2PI(_pb_icm[i], _b_icm[i], factor);

                _v_icm[i] = std::pow(-1,factor)*_v_icm[i]; // change sign to fit with steering

                _damp_icm[i] = std::tanh(std::fabs(_v_icm[i])/_treshhold_icm);

                _b_icm[i] = _pb_icm[i] + _damp_icm[i] * (_b_icm[i] - _pb_icm[i]);
                _v_icm[i] = v_last + _damp_icm[i] * (_v_icm[i] - v_last);

                factor = mwoibn::eigen_utils::limitHalfPi(_b[i],_b_icm[i]);
                factor = limit2PI_v(_b[i], _b_icm[i], factor);
                _v_icm[i] = std::pow(-1,factor)*_v_icm[i]; // change sign to fit with steering

        }
}

void mgnss::higher_level::SteeringReactif::_velSP(int i){
        _v_sp[i] = _K_v*_plane_ref.norm();
}

void mgnss::higher_level::SteeringReactif::_PT(int i)
{
        // Desired state

        _pb_sp[i] = _b_sp[i];
        double v_last = _v_sp[i];

        _plane_ref.noalias() = _plane.getReferenceError(i).head(2); // size 2 // get_reference state and old support?

        _steerSP(i);
        _velSP(i);

        //_v_sp[i] = _v_icm[i]*_v_sp[i];

        int factor = mwoibn::eigen_utils::limitHalfPi(_pb_sp[i],_b_sp[i]);
        factor = limit2PI(_pb_sp[i], _b_sp[i], factor);

        //limit2PI(_b_sp[i], _pb_sp[i]);
        _v_sp[i] = std::pow(-1,factor)*_v_sp[i]; // change sign to fit with steering

        _damp_sp[i] = std::tanh(std::fabs(_v_sp[i]) /_treshhold_sp);
        _b_sp[i] = _pb_sp[i] + _damp_sp[i] *(_b_sp[i] - _pb_sp[i]);
        _v_sp[i] = v_last + _damp_sp[i] * (_v_sp[i] - v_last);

        factor = mwoibn::eigen_utils::limitHalfPi(_b[i],_b_sp[i]);
        factor = limit2PI_v(_b[i], _b_sp[i], factor);
        _v_sp[i] = std::pow(-1,factor)*_v_sp[i]; // change sign to fit with steering

}
