#include "mgnss/controllers/wheeled_references_v2.h"

void mwoibn::SupportPolygon2::setCurrent(double x, double y)
{

        for (int i = 0; i < 4; i++)
                _current.segment<2>(3 * i) << _scale[3 * i] * x, _scale[3 * i + 1] * y;
}
void mwoibn::SupportPolygon2::setDesired(double x, double y)
{

        for (int i = 0; i < 4; i++)
                _desired.segment<2>(3 * i) << _scale[3 * i] * x, _scale[3 * i + 1] * y;
}
void mwoibn::SupportPolygon2::setBase(double x, double y)
{

        for (int i = 0; i < 4; i++)
                _base.segment<2>(3 * i) << _scale[3 * i] * x, _scale[3 * i + 1] * y;
}

void mwoibn::SupportPolygon2::nextStep()
{
        for (int i = 0; i < 4; i++)
        {
                _current.segment<2>(3 * i) << _scale[3 * i] * _r * std::cos(_t),
                -_scale[3 * i + 1] * _r * std::sin(_t);
        }

        _current += _base;
}

bool mwoibn::SupportPolygon2::moveToStart(double t, double step)
{

        setDesired(t);

        moveToStart(step);
}

void mwoibn::SupportPolygon2::setDesired(double t){

        for (int i = 0; i < 4; i++)
        {
                _desired.segment<2>(3 * i) << _scale[3 * i] * _r * std::cos(t),
                -_scale[3 * i + 1] * _r * std::sin(t);
        }

        _desired += _base;
}

bool mwoibn::SupportPolygon2::moveToStart(double step)
{
        bool done = true;
        _error.noalias() = _current - _desired;
        int k = 0;
        for (int i = 0; i < _error.size(); i++)
        {
                if (std::fabs(_error[i]) < step)
                        _current[k] = _desired[k];
                else if (_error[i] > 0)
                {
                        done = false;
                        _current[k] -= step;
                }
                else
                {
                        _current[k] += step;
                        done = false;
                }
                k++;
                if(!(i%2)) k++;

                std::cout << "i " << i << "\tk " << k;
                std::cout << "\t!(i%2) " << !(i%2) << std::endl;
        }

        return done;
}

void mwoibn::SupportPolygon2::setHeight(int i, double z){
        _desired[3*i+2] = z;
}
