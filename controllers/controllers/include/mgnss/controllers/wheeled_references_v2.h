#ifndef PROGRAMS_WHEELED_REFERENCES_2_H
#define PROGRAMS_WHEELED_REFERENCES_2_H

#include <mwoibn/common/types.h>
#include "mgnss/controllers/wheeled_references.h"

namespace mwoibn
{

class SupportPolygon2 : public Reference
{

public:
SupportPolygon2(double x, double y, double z) : Reference(12) {
        setCurrent(x, y);
        _error.setZero(8);
        for(int i = 0; i < 4; i++)
                setHeight(i, z);
}
SupportPolygon2() : Reference(12) {
        _error.setZero(8);
}
virtual ~SupportPolygon2() {
}

bool update()
{
        bool done = false;

        if (_motion == SUPPORT_MOTION::STOP) {
                done = true;
        }
        else if (_motion == SUPPORT_MOTION::DIRECT) {
                done = moveToStart(0.0005);
        }
        else if (_motion == SUPPORT_MOTION::CIRCULAR) {
                done = limitedStep();
                nextStep();
        }

        return done;
}

void changeMotion(SUPPORT_MOTION motion) {
        _motion = motion;
}
void changeState(SUPPORT_STATE state) {
        _state = state;
}
bool initMotion(SUPPORT_MOTION motion, SUPPORT_STATE state)
{
        _motion = motion;
        _state = state;
        initMotion();
}

bool initMotion()
{
        if (_motion == SUPPORT_MOTION::DIRECT)
        {
                if (_state == SUPPORT_STATE::DEFAULT)
                {
                        setDesired(0.50, 0.22);
                        return true;
                }
                if (_state == SUPPORT_STATE::MAMMAL)
                {

                        setDesired(-0.17453333);
                        return true;
                }
                if (_state == SUPPORT_STATE::TO_CIRCLE)
                {
                        resetAngle();
                        std::cout << _t << std::endl;
                        for (int i = 0; i < 4; i++)
                        {
                                _desired.segment<2>(3 * i) << _scale[3 * i] * _r * std::cos(_t),
                                -_scale[3 * i + 1] * _r * std::sin(_t);
                        }

                        _desired += _base;
                        return true;
                }
        }
        if (_motion == SUPPORT_MOTION::CIRCULAR)
        {
                resetAngle();

                if (_state == SUPPORT_STATE::SPIDER)
                {
                        setNegative();
                        return true;
                }
                if (_state == SUPPORT_STATE::MAMMAL)
                {
                        setPositive();
                        return true;
                }
        }
        return false;
}

virtual void setHeight(int i, double z);

virtual void setCurrent(double x, double y);
virtual void setBase(double x, double y);
virtual void setDesired(double x, double y);

virtual void setDesired(double t);
using Reference::setCurrent;
using Reference::setBase;
using Reference::setDesired;
using Reference::get;
void resetAngle() {
        _t = std::atan2(-_current[1] + _base[1], _current[0] - _base[0]);
}

mwoibn::VectorN get(int i) {
        return _current.segment<2>(2 * i);
}

virtual void nextStep();

bool moveToStart(double t, double step);
bool moveToStart(double step);


protected:
SUPPORT_MOTION _motion = SUPPORT_MOTION::STOP;
SUPPORT_STATE _state = SUPPORT_STATE::DEFAULT;
std::vector<int> _scale = {1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1};
mwoibn::VectorN _error;
};
}

#endif // WHEELED_MOTION_H
