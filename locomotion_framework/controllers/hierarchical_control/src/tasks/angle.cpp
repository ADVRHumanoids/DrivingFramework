#include "mwoibn/hierarchical_control/tasks/angle.h"



mwoibn::hierarchical_control::tasks::Angle::Angle(robot_class::angles::Basic& angle, mwoibn::robot_class::Robot& robot)
        : BasicTask(), _angle(angle)
{
        _init(1, robot.getDofs());
}

void mwoibn::hierarchical_control::tasks::Angle::updateError()
{
        _last_error.noalias() = _error;

        _angle.update();
        _error[0] = _ref - _angle.get();

        eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering
}

double mwoibn::hierarchical_control::tasks::Angle::getCurrent(){
        return _angle.get();
}

void mwoibn::hierarchical_control::tasks::Angle::updateJacobian() {

        _last_jacobian.noalias() = _jacobian;
        _jacobian.noalias() = -_angle.getJacobian();
}

double mwoibn::hierarchical_control::tasks::Angle::getReference() const {
        return _ref;
}

void mwoibn::hierarchical_control::tasks::Angle::setReference(double reference) {
        _ref = reference;
}


mwoibn::hierarchical_control::tasks::SoftAngle::SoftAngle(robot_class::angles::Basic& angle, mwoibn::robot_class::Robot& robot)
        : Angle(angle, robot)
{
        _resteer = false;
}


void mwoibn::hierarchical_control::tasks::SoftAngle::updateError()
{
        _last_error.noalias() = _error;

        _resteer = false;
        _angle.update();

        mwoibn::eigen_utils::wrapToPi(_ref);

        _error[0] = _ref - _angle.get();

        _limit2PI();

        mwoibn::eigen_utils::wrapToPi(_error[0]);
        mwoibn::eigen_utils::wrapToPi(_ref);

        _reverse( 50*mwoibn::PI/180);
        _saturation(30*mwoibn::PI/180);

}

void mwoibn::hierarchical_control::tasks::SoftAngle::_reverse(double limit){



        if ( _error[0] > 0 &&  std::fabs(_error[0] - mwoibn::PI) < limit) {
                _error[0]-= mwoibn::PI;
                _resteer = true;

        }

        else if (_error[0] < 0 &&  std::fabs(_error[0] + mwoibn::PI) < limit) {
                _error[0]+= mwoibn::PI;
                _resteer = true;
        }
}

void mwoibn::hierarchical_control::tasks::SoftAngle::_saturation(double limit){
        if (_error[0] > limit)
                _error[0] = limit;
        else if (_error[0] < -limit)
                _error[0] = -limit;
}

bool mwoibn::hierarchical_control::tasks::SoftAngle::resteer(){
        return _resteer;
}

void mwoibn::hierarchical_control::tasks::SoftAngle::_limit2PI(){

        if(_error[0] - _last_error[0] > mwoibn::PI) {
                _ref -= mwoibn::TWO_PI;
                _error[0] = _ref - _angle.get();
                _limit2PI();
        }
        else if (_last_error[0] - _error[0] > mwoibn::PI) {
                _ref += mwoibn::TWO_PI;
                _error[0] = _ref - _angle.get();
                _limit2PI();
        }
}
