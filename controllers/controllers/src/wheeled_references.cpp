#include <mgnss/controllers/wheeled_references.h>

void mwoibn::SupportPolygon::setCurrent(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _current.segment<2>(2 * i) << _scale[2 * i] * x, _scale[2 * i + 1] * y;
}
void mwoibn::SupportPolygon::setDesired(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _desired.segment<2>(2 * i) << _scale[2 * i] * x, _scale[2 * i + 1] * y;
}
void mwoibn::SupportPolygon::setBase(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _base.segment<2>(2 * i) << _scale[2 * i] * x, _scale[2 * i + 1] * y;
}

void mwoibn::SupportPolygon::nextStep()
{
  for (int i = 0; i < 4; i++)
  {
    _current.segment<2>(2 * i) << _scale[2 * i] * _r * std::cos(_t),
        -_scale[2 * i + 1] * _r * std::sin(_t);
  }

  _current += _base;
}

bool mwoibn::SupportPolygon::moveToStart(double t, double step)
{

  setDesired(t);

  moveToStart(step);
}

void mwoibn::SupportPolygon::setDesired(double t){

  for (int i = 0; i < 4; i++)
  {
    _desired.segment<2>(2 * i) << _scale[2 * i] * _r * std::cos(t),
        -_scale[2 * i + 1] * _r * std::sin(t);
  }

  _desired += _base;
}

bool mwoibn::SupportPolygon::moveToStart(double step)
{
  bool done = true;
  _error.noalias() = _current - _desired;
  for (int i = 0; i < _error.size(); i++)
  {
    if (std::fabs(_error[i]) < step)
      _current[i] = _desired[i];
    else if (_error[i] > 0)
    {
      done = false;
      _current[i] -= step;
    }
    else
    {
      _current[i] += step;
      done = false;
    }
  }

  return done;
}
