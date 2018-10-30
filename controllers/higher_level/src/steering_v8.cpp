#include "mgnss/higher_level/steering_v8.h"

mgnss::higher_level::Steering8::Steering8(
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

        _treshhold_icm = margin_icm/dt;
        _treshhold_sp = margin_sp/dt;

}

void mgnss::higher_level::Steering8::_resetTreshhold(){
        _treshhold = _treshhold*_dt;

        _treshhold_icm = _treshhold_icm*_dt;
        _treshhold_sp = _treshhold_sp*_dt;
}

void mgnss::higher_level::Steering8::_computeTreshhold(){
        _treshhold = _treshhold/_dt;

        _treshhold_icm = _treshhold_icm/_dt;
        _treshhold_sp = _treshhold_sp/_dt;
//  std::cout << _treshhold << "\t" << _treshhold_icm << "\t" << _treshhold_sp << std::endl;
}


void mgnss::higher_level::Steering8::_merge(int i){
        if(_resteer[i]) {
                _b[i] -= mwoibn::PI;
                mwoibn::eigen_utils::wrapToPi(_b[i]);
        } // change velocity sign

        _pb[i] = _b[i];

        _v[i] = _computeVelocity(i);
        _damp[i] = std::tanh(std::fabs(_v[i]*_v_icm[i]) /_treshhold/ _treshhold_icm);


        double scale, check;
        if(_v_sp[i] == 0 && _v_icm[i] == 0)
          scale = 0;
        else{
//          check = std::fabs(_K_icm*_v_icm[i])/std::fabs(_K_sp*_v_sp[i]);
//          scale = (check > 50 || std::fabs(_v_icm[i]) == 0) ? std::fabs(_v_icm[i]) : check/50;

            scale = (std::fabs(_v_icm[i]) < 0.1) ? std::fabs(1000*_v_icm[i]) : 1;

        }

        _b[i] = std::atan2(  _K_icm * _v_icm[i]  * std::sin(_b_icm[i]) +
                             _K_sp  * scale * _v_sp[i]   * std::sin(_b_sp[i]),
                             _K_icm * _v_icm[i]  * std::cos(_b_icm[i]) +
                             _K_sp  * scale * _v_sp[i]   * std::cos(_b_sp[i]));

        // std::cout.precision(5);
        // std::cout << std::fixed;
        // std::cout << i << "\t" << _v_icm[i] << std::endl;

        // std::cout << "Kicm\t" << _K_icm * _v_icm[i] << ", Ksp\t" << _K_sp * _v_sp[i]
        //           << ", Ksp v_sp\t" << _K_sp * scale * _v_sp[i]
        //           << ", scale\t" << scale
        //           << ", K ratio\t" << std::fabs(_K_icm*_v_icm[i])/std::fabs(_K_sp*_v_sp[i])
        //           << std::endl;



        _raw[i] = _b[i];
        _limited[i] = _b[i];

        limit2PI(_pb[i], _limited[i]);

        _b[i] = _pb[i] + _damp[i] * (_limited[i] - _pb[i]); // do give the result in the world frame

        _b_st[i] = _b[i] + _heading;
}

void mgnss::higher_level::Steering8::_steerICM(int i, const mwoibn::Vector3 next_step){

        SteeringReference::_steerICM(i, next_step);

        _x += _contact_vel[3*i];
        _y += _contact_vel[3*i+1];
        _b_icm[i] = std::atan2(_y, _x); // this uses atan2 to avoid
                                        // singularities in a atan
                                        // implementation
}


void mgnss::higher_level::Steering8::_ICM(mwoibn::Vector3 next_step)
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

void mgnss::higher_level::Steering8::_velSP(int i){
        _v_sp[i] = _K_v*_plane_ref.norm();
}

void mgnss::higher_level::Steering8::_PT(int i)
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
