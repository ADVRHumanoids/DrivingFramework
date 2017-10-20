#ifndef PROGRAMS_WHEELED_REFERENCES_H
#define PROGRAMS_WHEELED_REFERENCES_H

#include <mwoibn/common/types.h>

namespace mwoibn
{

enum class SUPPORT_STATE
{
  DEFAULT = 0,
  MAMMAL = 1,
  SPIDER = 2,
};
enum class SUPPORT_MOTION
{
  STOP = 0,
  DIRECT = 1,
  CIRCULAR = 2,
};

/*
enum class BASE_DIRECTION
{
  POSITIVE = 0,
  NEGATIVE = 1,
};

enum class BASE_MOTION
{
  STOP = 0,
  HEADING = 1,
  FULL = 2,
};
*/

class Reference
{

public:
  Reference(int size): _size(size)
  {
    _current = mwoibn::VectorN::Zero(size);
    _base = mwoibn::VectorN::Zero(size);
    _desired = mwoibn::VectorN::Zero(size);
  }

  ~Reference() {}

  virtual void setCurrent(const VectorN& current) { _current.noalias() = current.head(_size); }
  virtual void setBase(const VectorN& base) { _base.noalias() = base.head(_size); }
  virtual void setDesired(const VectorN& desired) { _desired.noalias() = desired.head(_size); }

  const mwoibn::VectorN& get() { return _current; }

  void setLowerLimit(double t)
  {
    _t_min = t;
    if (_direction == -1)
      _limit = t;
  }

  void setAngle(double t) { _t = t; }
  void setStep(double step) { _step = step; }

  void setUpperLimit(double t)
  {
    _t_max = t;
    if (_direction == 1)
      _limit = t;
  }

  void setRadious(double r) { _r = r; }

  virtual void nextStep() = 0;

  void changeDirection()
  {
    _direction = -_direction;
    if (_direction == 1)
      _limit = _t_max;
    else
      _limit = _t_min;
  }

  void setPositive()
  {
    _direction = 1;
    _limit = _t_max;
  }
  void setNegative()
  {
    _direction = -1;
    _limit = _t_min;
  }
  void step() { _t += _direction * _step; }
  bool limitedStep()
  {

    if (std::fabs(_t - _limit) > (_step))
    {
      _t += _direction * _step;
      return false;
    }
    else if (_direction == 1){
      _t = _t_max;
    std::cout << "current\t" << _t << ",\t limit\t" << _limit << std::endl;
	}
    else{
      _t = _t_min;
    std::cout << "current\t" << _t << ",\t limit\t" << _limit << std::endl;
	}

    return true;
  }

  bool isDone() { return (std::fabs(_t - _limit) < _step); }

protected:
  mwoibn::VectorN _current, _base, _desired;
  int _direction = 1, _size = 0;
  double _t_max, _t_min, _r, _t, _step = 0.002, _limit;
};

class Pose : public Reference
{

public:
  Pose() : Reference(2) {}
  ~Pose() {}
  void nextStep()
  {
    _current << _r* std::cos(_t - _offset) + _base[0],
        _r * std::sin(_t - _offset) + _r + _base[1];
  }

protected:
  double _offset = 1.57079632679;
};

class ScalarRef : public Reference
{

public:
  ScalarRef() : Reference(1) {}
  ~ScalarRef() {}
  virtual void setCurrent(double current) { _current[0] = current; }
  virtual void setBase(double base) { _base[0] = base; }
  virtual void setDesired(double desired) { _desired[0] = desired; }

  void nextStep() { _current[0] = _t; }
  void moveTangential()
  {
    _current[0] = std::atan2(std::sin(_t), std::cos(_t)) + _base[0];
  }
};

class Base
{

public:
  Base() {_state << 0,0,0; _omega = 0;}
  ~Base() {}

  void set(mwoibn::Vector3 velocity, double omega)
  {

    _state = velocity;
    _omega = omega;

 }
 
  void setX(double v_x)
  {
    _state[0] = v_x;
  }
  void setY(double v_y)
  {
    _state[1] = v_y;
  }
  void setZ(double v_z)
  {
    _state[2] = v_z;
  }
    void setHeading(double omega)
  {
    _omega = omega;
  }
 
 void setPose(double v_x, double v_y, double omega)
  {

    _state[0] = v_x;
    _state[1] = v_y;
    _omega = omega;

 }

    const double& getHeading()
  {
    return _omega;
  }
 
   const mwoibn::Vector3& getPosition()
  {
    return _state;
  }
 
  const mwoibn::Vector3& get()
  {

    _returner.head(2) = _state.head(2);
    _returner[2] = _omega;

    return _returner;
  }

protected:
  mwoibn::Vector3 _state, _returner;
  double _omega;

};

class SupportPolygon : public Reference
{

public:
  SupportPolygon(double x, double y) : Reference(8) { setCurrent(x, y); _error.setZero(8);}
  SupportPolygon() : Reference(8) {_error.setZero(8);}
  ~SupportPolygon() {}

  bool update()
  {
    bool done = false;

    if (_motion == SUPPORT_MOTION::STOP){
//      std::cout << "support STOP" << std::endl;

      done = true;
    }
    else if (_motion == SUPPORT_MOTION::DIRECT){
//      std::cout << "support DIRECT" << std::endl;

      done = moveToStart(0.0005);
    }
    else if (_motion == SUPPORT_MOTION::CIRCULAR){
//      std::cout << "support CIRCULAR" << std::endl;

      done = limitedStep();
      nextStep();
    }
    return done;
  }

  void changeMotion(SUPPORT_MOTION motion) { _motion = motion; }
  void changeState(SUPPORT_STATE state) { _state = state; }
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
//      std::cout << "support DIRECT" << std::endl;

      if (_state == SUPPORT_STATE::DEFAULT)
      {
//        std::cout << "support DEFAULT" << std::endl;

        setDesired(0.50, 0.22);
        // support.setAngle(-0.17453333);
        return true;
      }
      if (_state == SUPPORT_STATE::MAMMAL)
      {
//        std::cout << "support MAMMAL" << std::endl;

        setAngle(-0.17453333);
        setDesired(-0.17453333);
        return true;
      }
    }
    if (_motion == SUPPORT_MOTION::CIRCULAR)
    {
//      std::cout << "support CIRCULAR" << std::endl;

      if (_state == SUPPORT_STATE::SPIDER)
      {
//        std::cout << "support SPIDER" << std::endl;

        setNegative();
        return true;
      }
      if (_state == SUPPORT_STATE::MAMMAL)
      {
//        std::cout << "support MAMMAL" << std::endl;

        setPositive();
        return true;
      }
    }
    return false;
  }

  virtual void setCurrent(double x, double y);
  virtual void setBase(double x, double y);
  virtual void setDesired(double x, double y);

  virtual void setDesired(double t);
  using Reference::setCurrent;
  using Reference::setBase;
  using Reference::setDesired;
  using Reference::get;

  mwoibn::VectorN get(int i) { return _current.segment<2>(2 * i); }

  virtual void nextStep();

  bool moveToStart(double t, double step);
  bool moveToStart(double step);

protected:
  SUPPORT_MOTION _motion = SUPPORT_MOTION::STOP;
  SUPPORT_STATE _state = SUPPORT_STATE::DEFAULT;
  std::vector<int> _scale = {1, 1, 1, -1, -1, 1, -1, -1};
  mwoibn::VectorN _error;
};
}

#endif // WHEELED_MOTION_H
