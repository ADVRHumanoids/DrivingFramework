#include <mgnss/controllers/wheeled_references_v3.h>

void mwoibn::SupportPolygon3::setCurrent(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setCurrent(_scale[3 * i] * x, _scale[3 * i + 1] * y);
}
void mwoibn::SupportPolygon3::setDesired(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setDesired(_scale[3 * i] * x, _scale[3 * i + 1] * y);
}
void mwoibn::SupportPolygon3::setBase(double x, double y)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setBase(_scale[3 * i] * x, _scale[3 * i + 1] * y);
}

void mwoibn::SupportPolygon3::setCurrent(double x, double y, double z)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setCurrent(_scale[3 * i] * x, _scale[3 * i + 1] * y,
                            _scale[3 * i + 2] * z);
}
void mwoibn::SupportPolygon3::setDesired(double x, double y, double z)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setDesired(_scale[3 * i] * x, _scale[3 * i + 1] * y,
                            _scale[3 * i + 2] * z);
}
void mwoibn::SupportPolygon3::setBase(double x, double y, double z)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setBase(_scale[3 * i] * x, _scale[3 * i + 1] * y,
                         _scale[3 * i + 2] * z);
}

void mwoibn::SupportPolygon3::nextStep()
{
  for (int i = 0; i < 4; i++)
  {
    _contacts[i].setCurrent( _contacts[i].getBase()[0] + _scale[3 * i] * _r * std::cos(_t),
       _contacts[i].getBase()[1] -_scale[3 * i + 1] * _r * std::sin(_t));
  }

  //  _current += _base;
}

void mwoibn::SupportPolygon3::setDesired(double t)
{

  for (int i = 0; i < 4; i++)
  {
    _contacts[i].setDesired( _contacts[i].getBase()[0] + _scale[3 * i] * _r * std::cos(t),
       _contacts[i].getBase()[1] -_scale[3 * i + 1] * _r * std::sin(t));
  }

  //  _desired += _base;
}

void mwoibn::SupportPolygon3::setCurrent(mwoibn::VectorN current)
{

  for (int i = 0; i < 4; i++)
    _contacts[i].setCurrent(current[3 * i], current[3 * i + 1], current[3 * i + 2]);
}

void mwoibn::SupportPolygon3::setBase(mwoibn::VectorN base)
{
  for (int i = 0; i < 4; i++)
    _contacts[i].setBase(base[3 * i], base[3 * i + 1], base[3 * i + 2]);
}

void mwoibn::SupportPolygon3::setDesired(mwoibn::VectorN desired)
{
  for (int i = 0; i < 4; i++)
    _contacts[i].setDesired(desired[3 * i], desired[3 * i + 1], desired[3 * i + 2]);
}

const mwoibn::VectorN& mwoibn::SupportPolygon3::get()
{

  for (int i = 0; i < 4; i++)
    _full_state.segment<3>(3 * i) = _contacts[i].get();

//  std::cout << _full_state.transpose() << std::endl;
  return _full_state;
}

bool mwoibn::SupportPolygon3::moveToStart(double t, double step)
{

  setDesired(t);

  moveToStart(step);
}

bool mwoibn::SupportPolygon3::moveToStart(double step)
{
  bool done = true;

  for (int i = 0; i < 4; i++)
  {
    _error.noalias() = _contacts[i].get() - _contacts[i].getDesired();

    mwoibn::VectorN move = _contacts[i].get();

    for (int k = 0; k < 2; k++)
    {
      if (std::fabs(_error[k]) < step)
        move[k] = _contacts[i].getDesired()[k];
      else if (_error[k] > 0)
      {
        done = false;
        move[k] -= step;
      }
      else
      {
        move[k] += step;
        done = false;
      }
    }
//    std::cout << "\t" << i<< "\t" << _contacts[i].get().transpose();

    _contacts[i].setCurrent(move[0], move[1]);

//    std::cout << "\t" << i<< "\t" << _contacts[i].getDesired().transpose() << std::endl;

  }

  return done;
}

bool mwoibn::SupportPolygon3::initMotion()
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

      for (int i = 0; i < 4; i++)
      {
        _contacts[i].setDesired( _contacts[i].getBase()[0] + _scale[3 * i] * _r * std::cos(_t),
           _contacts[i].getBase()[1] -_scale[3 * i + 1] * _r * std::sin(_t));
      }

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
      setDesired(-0.17453333);
      setPositive();

      return true;
    }
  }
  return false;
}

bool mwoibn::SupportPolygon3::update()
{
  bool done = false;

//  std::cout << 1 << std::endl;
  if (_motion == SUPPORT_MOTION::STOP){
//    std::cout << 2 << std::endl;
    done = true;
  }
  else if (_motion == SUPPORT_MOTION::DIRECT){
//    std::cout << 3 << std::endl;

    done = moveToStart(0.0005);
  }
  else if (_motion == SUPPORT_MOTION::CIRCULAR){
//    std::cout << 4 << std::endl;

    done = limitedStep();
    nextStep();
  }
//  std::cout << 5 << std::endl;

  for(int i = 0; i < _contacts.size(); i++)
    _contacts[i].update();
  return done;
}
