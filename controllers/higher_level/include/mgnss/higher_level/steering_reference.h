#ifndef __MGNSS_HIGHER_LEVEL_STEERING_REFERENCE_H
#define __MGNSS_HIGHER_LEVEL_STEERING_REFERENCE_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/hierarchical_control/tasks/contact_point_tracking_task.h>
#include <mwoibn/robot_points/point.h>

namespace mgnss {

namespace higher_level
{


inline void limit(double b_ref, double& b)
{

        mwoibn::eigen_utils::wrapToPi(b);
        mwoibn::eigen_utils::wrapToPi(b_ref);

        b = ((std::fabs(b - b_ref) < mwoibn::HALF_PI) ||
             (std::fabs(b - b_ref) > 3 * mwoibn::HALF_PI))
            ? b
            : (b - b_ref < 0) ? b + mwoibn::PI : b - mwoibn::PI;
}

inline void limit2(double b_ref, double& b)
{
        b +=   3.14159265 * (std::floor((b_ref + 1.57079632) / 3.14159265) -  std::floor((b + 1.57079632) / 3.14159265));
}


inline void limit(const mwoibn::VectorN& b_ref, mwoibn::VectorN& b)
{
        for (int i = 0; i < b.size(); i++)
                limit(b_ref[i], b[i]);
}

inline void jointLimits(double& b, double max)
{

        if (b > max) // this is a temporary solution until they are joint
                     // limits implemented
                b -= 3.1415926;
        else if (b < -max) // this is a temporary solution until they are
                           // joint limits implemented
                b += 3.1415926;
}

inline void jointLimits(mwoibn::VectorN& b, double max)
{

        for (int i = 0; i < b.size(); i++)
                jointLimits(b[i], max);
}


class SteeringReference
{

public:
SteeringReference(mwoibn::robot_class::Robot& robot,
                  mwoibn::hierarchical_control::tasks::ContactPointTracking& plane,
                  double K_icm, double K_sp, double dt, double margin = 0.04,
                  double max = 2.79252680) : _plane(plane), _K_icm(K_icm), _K_sp(K_sp), _dt(dt),
        _state(robot.state.position.get())
{
        _v_icm.setZero(_size);
        _b_icm.setZero(_size);
        _v_sp.setZero(_size);
        _b_sp.setZero(_size);
        _b.setZero(_size);
        _v.setZero(_size);
        _damp.setZero(_size);
        _raw.setZero(_size);
        _limited.setZero(_size);
        _b_st.setZero(_size);
        _plane_ref.setZero(2);
        _resteer.setConstant(_size, false);
        _steer.setConstant(_size, false);
        _treshhold = margin/ dt;

}

virtual ~SteeringReference() {
}

const mwoibn::VectorN& get() {
        return _b_st;
}
void set(mwoibn::VectorN last) {
        _b_st = last;
}

virtual void setRate(double dt)
{
        _resetTreshhold();
        _dt = dt;
        _computeTreshhold();
}


const mwoibn::VectorN& getICM() {
        return _b_icm;
}
const mwoibn::VectorN& getSP() {
        return _b_sp;
}
const mwoibn::VectorN& getLimited() {
        return _limited;
}
const mwoibn::VectorN& vICM() {
        return _v_icm;
}
const mwoibn::VectorN& vSP() {
        return _v_sp;
}
const mwoibn::VectorN& v() {
        return _v;
}
const mwoibn::VectorN& damp() {
        return _damp;
}
const mwoibn::VectorN& getDampingSP() {
        return _damp_sp;
}
const mwoibn::VectorN& getDampingICM() {
        return _damp_icm;
}
const mwoibn::VectorN& getRaw() {
        return _raw;
}

void resteer(const mwoibn::VectorBool& steer)
{
        for (int i = 0; i < _resteer.size(); i++)
        {
//      if (!_steer[i])
                _resteer[i] = steer[i];
//      else
//        _resteer[i] = false;
        }
//    _steer.noalias() = steer;
}

virtual void compute(const mwoibn::Vector3 next_step){

        _plane.updateState();
        _heading = _plane.heading();
        _plane.updateError();

        _ICM(next_step); // returns a velue in a robot space

        _SPT(); // returns a velue in a robot space

        for (int i = 0; i < _size; i++) _merge(i);
}

static int limit2PI(double ref, double& st, int factor){

        if(st - ref > mwoibn::HALF_PI) {
                st -= mwoibn::PI;
                factor -= 1;
                limit2PI(ref, st, factor);
        }
        else if ((ref - st > mwoibn::HALF_PI)) {
                st += mwoibn::PI;
                factor += 1;
                limit2PI(ref, st, factor);
        }
        return factor;
}

static void limit2PI(double ref, double& st){

        if(st - ref > mwoibn::HALF_PI) {
                st -= mwoibn::PI;
                limit2PI(ref, st);
        }
        else if ((ref - st > mwoibn::HALF_PI)) {
                st += mwoibn::PI;
                limit2PI(ref, st);
        }
}

static int limit2PI_v(double ref, double& st, int factor){

        if(st - ref > mwoibn::HALF_PI + 10.0/180.0*mwoibn::PI) {
                st -= mwoibn::PI;
                factor -= 1;
                limit2PI(ref, st, factor);
        }
        else if ((ref - st > mwoibn::HALF_PI + 10.0/180.0*mwoibn::PI)) {
                st += mwoibn::PI;
                factor += 1;
                limit2PI(ref, st, factor);
        }
        return factor;
}

static void limit2PI_v(double ref, double& st){

        if(st - ref > mwoibn::HALF_PI + 10.0/180.0*mwoibn::PI) {
                st -= mwoibn::PI;
                limit2PI(ref, st);
        }
        else if ((ref - st > mwoibn::HALF_PI + 10.0/180.0*mwoibn::PI)) {
                st += mwoibn::PI;
                limit2PI(ref, st);
        }
}

static void limitPI(double ref, double& st){

        if(st - ref > mwoibn::HALF_PI/2) {
                st -= mwoibn::HALF_PI;
                limitPI(ref, st);
        }
        else if ((ref - st > mwoibn::HALF_PI/2)) {
                st += mwoibn::HALF_PI;
                limitPI(ref, st);
        }
}

protected:
mwoibn::hierarchical_control::tasks::ContactPointTracking& _plane;
double _dt, _max, _K_icm, _K_sp, _heading, _x, _y, _treshhold;
mwoibn::VectorN _damp_icm, _v_icm, _b_icm, _v_sp, _b_sp, _b, _b_st, _plane_ref, _damp_sp, _v, _damp, _raw, _limited;
const mwoibn::VectorN& _state;
mwoibn::VectorInt _dofs;
mwoibn::VectorBool _resteer, _steer;

int _size = 4;

virtual void _ICM(mwoibn::Vector3 next_step) = 0;

virtual void _SPT()
{
        for (int i = 0; i < _size; i++)
                _PT(i);
}
virtual void _PT(int i) = 0;

virtual void _merge(int i){
        _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                           _K_sp  * _v_sp[i] * std::sin(_b_sp[i]),
                           _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                           _K_sp  * _v_sp[i] * std::cos(_b_sp[i]));

}

virtual double _computeVelocity(int i){
        return std::fabs(_v_icm[i]*_v_icm[i] + _v_sp[i]*_v_sp[i] + 2*_v_sp[i]*_v_icm[i]*std::cos(_b_icm[i] - _b_sp[i]));

}

virtual void _steerICM(int i, const mwoibn::Vector3 next_step){
        _plane_ref.noalias() = _plane.getPointStateReference(i).head(2);

        _x = std::cos(_heading) * next_step[0];
        _x += std::sin(_heading) * next_step[1];
        _x -= _plane_ref[1] * next_step[2];
        _y = -std::sin(_heading) * next_step[0];
        _y += std::cos(_heading) * next_step[1];
        _y += _plane_ref[0] * next_step[2];


        _b_icm[i] = std::atan2(_y, _x); // this uses atan2 to avoid
                                        // singularities in a atan
                                        // implementation
}

virtual void _velICM(int i){
        _v_icm[i] = _x * std::cos(_b_icm[i]);
        _v_icm[i] += _y * std::sin(_b_icm[i]);
}

virtual void _steerSP(int i){
        _b_sp[i] = std::atan2(_plane_ref[1], _plane_ref[0]);
}

virtual void _velSP(int i){
        _v_sp[i] = _plane_ref.norm() / _dt;
}
virtual void _computeTreshhold() = 0;
virtual void _resetTreshhold() = 0;

};
}
}
#endif // PROGRAM_STEERING_H
