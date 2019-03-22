#ifndef __MGNSS_HIGHER_LEVEL_STEERING_SHAPE_H
#define __MGNSS_HIGHER_LEVEL_STEERING_SHAPE_H

#include "mgnss/higher_level/steering_reference.h"

namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *
 */
class SteeringShape : public SteeringReference
{

public:
SteeringShape(mwoibn::robot_class::Robot& robot,
                mwoibn::hierarchical_control::tasks::ContactPoint& plane, const mwoibn::VectorN& contact_vel,
                double K_icm, double K_sp, double K_v, double dt,
                double margin_icm, double margin_sp, double margin = 0.04, double max = 2.79252680);

virtual ~SteeringShape() {
}
virtual void compute(const mwoibn::Vector3 next_step, const mwoibn::VectorN& db_des){


        _plane.updateError();
        _heading = _plane.heading();

        _ICM(next_step, db_des); // returns a velue in a robot space

        _SPT(); // returns a velue in a robot space


        for (int i = 0; i < _size; i++) _merge(i);

        _last_state[0] = _plane.baseX(); // will it initialize correctly?
        _last_state[1] = _plane.baseY(); // will it initialize correctly?

        _last_state[2] = _plane.heading(); // will it initialize correctly?
}

protected:

virtual void _merge(int i);
mwoibn::VectorN _pb_icm, _pb_sp, _pb, _reactif_gain;
mwoibn::Vector3 _last_state;
double _K_v, _treshhold_icm, _treshhold_sp;
virtual void _ICM(mwoibn::Vector3 next_step, const mwoibn::VectorN& db_des);
virtual void _ICM(mwoibn::Vector3 next_step){}

const mwoibn::VectorN& _contact_vel;
//mwoibn::robot_class::Robot& _robot;

virtual void _PT(int i);

virtual void _velSP(int i);
virtual void _velICM(int i);
virtual void _computeTreshhold();
virtual void _resetTreshhold();
virtual void _steerICM(int i, const mwoibn::Vector3 next_step, const mwoibn::VectorN& db_des);

void _reactifGain(int i);

virtual double _computeVelocity(int i){
        return std::sqrt(_K_icm*_v_icm[i]*_K_icm*_v_icm[i] + _K_sp*_v_sp[i]*_K_sp*_v_sp[i] + 2*_K_sp*_v_sp[i]*_K_icm*_v_icm[i]*std::cos(_b_icm[i] - _b_sp[i]));
}

};
}
}
#endif // PROGRAM_STEERING_H
