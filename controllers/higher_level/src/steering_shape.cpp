#include "mgnss/higher_level/steering_shape.h"

mgnss::higher_level::SteeringShape::SteeringShape(
        mwoibn::robot_class::Robot& robot,
        mwoibn::hierarchical_control::tasks::ContactPoint& plane,
        const mwoibn::VectorN& contact_vel,
        double K_icm, double K_sp, double K_v, double dt,
        double margin_icm, double margin_sp, double margin, double max)
        : SteeringReference(robot, plane, K_icm, K_sp, dt, margin, max), _K_v(K_v), _contact_vel(contact_vel)
{
        _pb_icm.setZero(_size);
        _pb_sp.setZero(_size);
        _pb.setZero(_size);

        _damp_sp.setZero(_size);
        _damp_icm.setZero(_size);

        _reactif_gain.setZero(_size);
        _current.setZero(_size);
        _treshhold_icm = margin_icm/dt;
        _treshhold_sp = margin_sp/dt;

        _plane.updateState();
        _last_state[0] = _plane.baseX(); // will it initialize correctly?
        _last_state[1] = _plane.baseY(); // will it initialize correctly?

        _last_state[2] = _plane.heading(); // will it initialize correctly?

}

void mgnss::higher_level::SteeringShape::_resetTreshhold(){
        _treshhold = _treshhold*_dt;

        _treshhold_icm = _treshhold_icm*_dt;
        _treshhold_sp = _treshhold_sp*_dt;
}

void mgnss::higher_level::SteeringShape::_computeTreshhold(){
        _treshhold = _treshhold/_dt;

        _treshhold_icm = _treshhold_icm/_dt;
        _treshhold_sp = _treshhold_sp/_dt;
}


void mgnss::higher_level::SteeringShape::_merge(int i){
        if(_resteer[i]) {
                _b[i] -= mwoibn::PI;
                mwoibn::eigen_utils::wrapToPi(_b[i]);
        } // change velocity sign

        _pb[i] = _b[i];
        _pb[i] = _current[i];

        _v[i] = _computeVelocity(i);
        _reactifGain(i);




        double scale = (std::fabs(_reactif_gain[i]) < 0.1 && std::fabs(_v_sp[i]*_dt) < 0.0002) ? (std::fabs(1000*_reactif_gain[i])+std::fabs(_v_sp[i])) : 1;

        _b[i] = std::atan2(  _K_icm * _v_icm[i]  * std::sin(_b_icm[i]) +
                             _K_sp  * scale * _v_sp[i]   * std::sin(_b_sp[i]),
                             _K_icm * _v_icm[i]  * std::cos(_b_icm[i]) +
                             _K_sp  * scale * _v_sp[i]   * std::cos(_b_sp[i]));
                             std::cout << std::fixed;
                             std::cout << std::setprecision(8);
        // std::cout << i << "\t_K_icm: " << _K_icm << "\t_K_sp: " << _K_sp << "\t_v_icm: " << _v_icm[i] << "\t_v_sp: " << _v_sp[i] <<  "\t_b_icm: " << _b_icm[i] << "\t_b_sp: " << _b_sp[i] << "\trg: " << _reactif_gain[i] << "\t_K_sp|_r_gain|_v_sp" << _K_sp  * std::fabs(_reactif_gain[i]) * _v_sp[i] << std::endl;
        // std::cout << i << "\t"
        //               << _b[i];
        //               << "\t"
        //               << _K_icm * _v_icm[i]
        //               << "\t"
        //               << _K_icm * _v_icm[i]  * std::sin(_b_icm[i])
        //               << "\t"
        //               << _K_icm * _v_icm[i]  * std::cos(_b_icm[i])
        //               << "\t"
        //               << _K_sp  * std::fabs(_reactif_gain[i]) * _v_sp[i]/_dt
        //               << "\t"
        //               << _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::sin(_b_sp[i])
        //               << "\t"
        //               << _K_sp  * std::fabs(_reactif_gain[i]/_dt) * _v_sp[i]   * std::cos(_b_sp[i])
        //               << std::endl;

        _raw[i] = _b[i];
        _limited[i] = _b[i];

        limit2PI(_pb[i], _limited[i]);

        double old_scale = scale;
        scale = (std::fabs(_v_icm[i]) < 0.1 && scale < 1) ? scale + std::fabs(1000*_v_icm[i]) : 1;

        _damp[i] = std::tanh(std::fabs( _v[i]*scale /_treshhold/(_treshhold_sp + _treshhold_icm)));



        _b[i] = _pb[i] + _damp[i] * (_limited[i] - _pb[i]); // do give the result in the world frame
        // std::cout << i << "\t" << _b[i] << "\t" << _damp[i] << std::endl;

        _b_st[i] = _b[i] + _heading;

        // std::cout << "Kicm\t" << _K_icm * _v_icm[i] << ", Ksp\t" << _v_sp[i]
        //           << ", Ksp v_sp\t" << _K_sp * scale * _v_sp[i]
        //           << ", scale\t" << scale << ", old scale\t" << old_scale
        //           << ", damp\t" << _damp[i]
        //           << ", Ksp\t" << _K_sp
        //           << ", _b_st[i]\t" << _b_st[i]
        //           << "icm\t" << std::atan2(  _K_icm * _v_icm[i]  * std::sin(_b_icm[i]), _K_icm * _v_icm[i]  * std::cos(_b_icm[i]))
        //           << "sp\t" << std::atan2(  _K_sp  * scale * _v_sp[i] * std::sin(_b_sp[i]), _K_sp  * scale * _v_sp[i]  * std::cos(_b_sp[i]))
        //           << std::endl;

        // std::cout << i << "_merge\t" << _v_sp[i]*_dt  << "\tscale\t" << scale << "\tcond1\t" << (std::fabs(_reactif_gain[i]) < 0.1) <<
                     // "\tcond2\t" << (std::fabs(_v_sp[i]*_dt) < 0.0002) << "\tcond\t" << (std::fabs(_reactif_gain[i]) < 0.1 && std::fabs(_v_sp[i]*_dt) < 0.0002) << std::endl;

}


// void mgnss::higher_level::SteeringShape::_merge(int i){
//         if(_resteer[i]) {
//                 _b[i] -= mwoibn::PI;
//                 mwoibn::eigen_utils::wrapToPi(_b[i]);
//         } // change velocity sign
//
//         _pb[i] = _b[i];
//
//         _v[i] = _computeVelocity(i);
//         _damp[i] = std::tanh(std::fabs(_v[i]*_v_icm[i]) /_treshhold/ _treshhold_icm);
//
//
//         double scale, check;
//         if(_v_sp[i] == 0 && _v_icm[i] == 0)
//           scale = 0;
//         else{
// //          check = std::fabs(_K_icm*_v_icm[i])/std::fabs(_K_sp*_v_sp[i]);
// //          scale = (check > 50 || std::fabs(_v_icm[i]) == 0) ? std::fabs(_v_icm[i]) : check/50;
//
//             scale = (std::fabs(_v_icm[i]) < 0.1) ? std::fabs(1000*_v_icm[i]) : 1;
//
//         }
//
//         _b[i] = std::atan2(  _K_icm * _v_icm[i]  * std::sin(_b_icm[i]) +
//                              _K_sp  * scale * _v_sp[i]   * std::sin(_b_sp[i]),
//                              _K_icm * _v_icm[i]  * std::cos(_b_icm[i]) +
//                              _K_sp  * scale * _v_sp[i]   * std::cos(_b_sp[i]));
//
//         // std::cout.precision(5);
//         // std::cout << std::fixed;
//         // std::cout << i << "\t" << _v_icm[i] << std::endl;
//
//
//         _raw[i] = _b[i];
//         _limited[i] = _b[i];
//
//         limit2PI(_pb[i], _limited[i]);
//
//         _b[i] = _pb[i] + _damp[i] * (_limited[i] - _pb[i]); // do give the result in the world frame
//
//         _b_st[i] = _b[i] + _heading;
//
//         std::cout << "Kicm\t" << _K_icm * _v_icm[i] << ", Ksp\t" << _v_sp[i]
//                   << ", Ksp v_sp\t" << _K_sp * scale * _v_sp[i]
//                   << ", scale\t" << scale
//                   << ", damp\t" << _damp[i]
//                   << ", Ksp\t" << _K_sp
//                   << ", _b_st[i]\t" << _b_st[i]
//                   << std::endl;
// }


void mgnss::higher_level::SteeringShape::_steerICM(int i, const mwoibn::Vector3 next_step, const mwoibn::VectorN& db_des){

        // SteeringReference::_steerICM(i, next_step);
        _plane_ref.noalias() = _plane.getPointStateReference(i).head(2);
        _b_st[i] = _current[i]; // set desired to current
        _b_sp[i] = _current[i]; // set desired to current

        // _b_icm[i] += db_des[i]; // this is switched to take only a desired steering, use the same way of computing velocity
        _b_icm[i] = _b_st[i] + db_des[i]; // this is switched to take only a desired steering, use the same way of computing velocity

}

void mgnss::higher_level::SteeringShape::_reactifGain(int i){

        double th = (_plane.heading() - _last_state[2]);
        mwoibn::eigen_utils::wrapToPi(th);

        _plane_ref.noalias() = _plane.getReferenceError(i).head(2);
        // _plane_ref += _plane.getVelocityReference(i).head<2>();

        double x = std::cos(_heading) * (_plane.baseX() - _last_state[0]);
        x += std::sin(_heading) * (_plane.baseY() - _last_state[1]);
        x -= _plane_ref[1] * th;

        double y = -std::sin(_heading) * (_plane.baseX() - _last_state[0]);
        y += std::cos(_heading) * (_plane.baseY() - _last_state[1]);
        y += _plane_ref[0] * th;

        double b = std::atan2(y, x);

        _reactif_gain[i] = x * std::cos(b);
        _reactif_gain[i] += y * std::sin(b);

        _reactif_gain[i] = _reactif_gain[i]/_dt;



}

void mgnss::higher_level::SteeringShape::_ICM(mwoibn::Vector3 next_step, const mwoibn::VectorN& db_des)
{

        _pb_icm.noalias() = _b_icm;
        _pb_icm = _current;
        for (int i = 0; i < _size; i++)
        {
                double v_last = _v_icm[i];

                _steerICM(i, next_step, db_des);
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

void mgnss::higher_level::SteeringShape::_velICM(int i){
        _x = _plane.getReferenceError(i)[0];
        _y = _plane.getReferenceError(i)[1];
        _v_icm[i] = _x * std::cos(_b_icm[i]);
        _v_icm[i] += _y * std::sin(_b_icm[i]);
        // _v_icm[i] = _plane.getError().norm(); // it is in a current steering frame I need it in the world frame?
}

void mgnss::higher_level::SteeringShape::_velSP(int i){
        _v_sp[i] = _K_v*_plane_ref.norm(); // this one assumes the _b_sp is in the +/- pi range
}

void mgnss::higher_level::SteeringShape::_PT(int i)
{
        // Desired state

        _pb_sp[i] = _b_sp[i];
        _pb_sp[i] = _current[i];

        double v_last = _v_sp[i];

        _plane_ref.noalias() = _plane.getReferenceError(i).head<2>();
        _plane_ref += _plane.getVelocityReference(i).head<2>(); // size 2 // get_reference state and old support?
        // std::cout << "getVelocityReference " << i << "\t" << _plane.getVelocityReference(i) << std::endl;

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
