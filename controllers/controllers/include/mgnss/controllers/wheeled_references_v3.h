#ifndef PROGRAMS_WHEELED_REFERENCES_2_H
#define PROGRAMS_WHEELED_REFERENCES_2_H

#include <mwoibn/common/types.h>
#include <mgnss/controllers/wheeled_references.h>

namespace mwoibn
{

class Contact
{

public:
  Contact(double x, double y, double z, double dt)
      : _vx(0), _vy(0), _vz(0), _dt(dt)
  {
    _ix = mwoibn::SUPPORT_INTERFACE::POSITION;
    _iy = mwoibn::SUPPORT_INTERFACE::POSITION;
    _iz = mwoibn::SUPPORT_INTERFACE::POSITION;
    _x.setCurrent(x);
    _y.setCurrent(y);
    _z.setCurrent(z);

  }

  Contact(double dt)
      : _vx(0), _vy(0), _vz(0), _dt(dt)
  {
    _ix = mwoibn::SUPPORT_INTERFACE::POSITION;
    _iy = mwoibn::SUPPORT_INTERFACE::POSITION;
    _iz = mwoibn::SUPPORT_INTERFACE::POSITION;
    _x.setCurrent(0);
    _y.setCurrent(0);
    _z.setCurrent(0);

  }

  ~Contact() {}

  void setVelX(double vx) { _vx = vx; }
  void setVelY(double vy) { _vy = vy; }
  void setVelZ(double vz) { _vz = vz; }

  void setModeX(mwoibn::SUPPORT_INTERFACE interface) { _ix = interface; }
  void setModeY(mwoibn::SUPPORT_INTERFACE interface) { _iy = interface; }
  void setModeZ(mwoibn::SUPPORT_INTERFACE interface) { _iz = interface; }

  void setCurrent(double x, double y, double z)
  {
    setCurrent(x, y);
    if (_iz == mwoibn::SUPPORT_INTERFACE::POSITION)
      _z.setCurrent(z);
  }

  void setCurrent(double x, double y)
  {
    if (_ix == mwoibn::SUPPORT_INTERFACE::POSITION)
      _x.setCurrent(x);
    if (_iy == mwoibn::SUPPORT_INTERFACE::POSITION)
      _y.setCurrent(y);
  }

  void setDesired(double x, double y, double z)
  {
    setDesired(x, y);
    _z.setDesired(z);
  }

  void setDesired(double x, double y)
  {
    _x.setDesired(x);
    _y.setDesired(y);
  }

  void setDesX(double x)
  {
    _x.setDesired(x);
  }

  void setDesY(double y)
  {
    _y.setDesired(y);
  }

  void setDesZ(double z)
  {
    _z.setDesired(z);
  }
  void setBase(double x, double y, double z)
  {
    setBase(x, y);
    _z.setBase(z);
  }

  void setBase(double x, double y)
  {
    _x.setBase(x);
    _y.setBase(y);
  }

  void update()
  {
    if (_ix == mwoibn::SUPPORT_INTERFACE::VELOCITY)
      _x.setCurrent(_x.get()[0] + _vx * _dt);
    if (_iy == mwoibn::SUPPORT_INTERFACE::VELOCITY)
      _y.setCurrent(_y.get()[0] + _vy * _dt);
    if (_iz == mwoibn::SUPPORT_INTERFACE::VELOCITY)
      _z.setCurrent(_z.get()[0] + _vz * _dt);
  }

  const mwoibn::Vector3& get()
  {
    _state << _x.get(), _y.get(), _z.get();
    return _state;
  }
  const mwoibn::Vector3& getBase()
  {
    _state << _x.getBase(), _y.getBase(), _z.getBase();
    return _state;
  }
  const mwoibn::Vector3& getDesired()
  {
    _state << _x.getDesired(), _y.getDesired(), _z.getDesired();
    return _state;
  }


protected:
  ScalarRef _x, _y, _z;
  double _vx, _vy, _vz, _dt;
  mwoibn::SUPPORT_INTERFACE _ix, _iy, _iz;
  mwoibn::Vector3 _state;
};

class SupportPolygon3: public Reference
{
public:
  SupportPolygon3(double x, double y, double z, double dt): Reference(1)
  {
    for (int i = 0; i < 4; i++)
      _contacts.push_back(mwoibn::Contact(dt));

    setCurrent(x, y, z);
    _error.setZero(2);
    _full_state.setZero(12);
  }

  bool update();

  void changeMotion(SUPPORT_MOTION motion) { _motion = motion; }
  void changeState(SUPPORT_STATE state) { _state = state; }
  bool initMotion(SUPPORT_MOTION motion, SUPPORT_STATE state)
  {
    _motion = motion;
    _state = state;
    initMotion();
  }

  bool initMotion();

  virtual void setCurrent(double x, double y);
  virtual void setBase(double x, double y);
  virtual void setDesired(double x, double y);

  virtual void setCurrent(double x, double y, double z);
  virtual void setBase(double x, double y, double z);
  virtual void setDesired(double x, double y, double z);

  virtual void setDesired(double t);

  virtual void setCurrent(mwoibn::VectorN current);
  virtual void setBase(mwoibn::VectorN base);
  virtual void setDesired(mwoibn::VectorN desired);

  virtual void setDesX(int i, double x){ _contacts[i].setDesX(x);}
  virtual void setDesY(int i, double y){ _contacts[i].setDesY(y);}
  virtual void setDesZ(int i, double z){ _contacts[i].setDesZ(z);}

  virtual void setVelX(int i, double vx) { _contacts[i].setVelX(vx); }
  virtual void setVelY(int i, double vy) { _contacts[i].setVelY(vy); }
  virtual void setVelZ(int i, double vz) { _contacts[i].setVelZ(vz); }

  void setModeX(int i, mwoibn::SUPPORT_INTERFACE interface){_contacts[i].setModeX(interface);}
  void setModeY(int i, mwoibn::SUPPORT_INTERFACE interface){_contacts[i].setModeY(interface);}
  void setModeZ(int i, mwoibn::SUPPORT_INTERFACE interface){_contacts[i].setModeZ(interface);}

  virtual const mwoibn::VectorN& get();

  void resetAngle()
  {
    _t = std::atan2(-_contacts[0].get()[1] + _contacts[0].getBase()[1],
                    _contacts[0].get()[0] - _contacts[0].getBase()[0]);
  }

  const mwoibn::Vector3& get(int i) { return _contacts[i].get(); }

  virtual void nextStep();

  bool moveToStart(double t, double step);
  bool moveToStart(double step);

protected:
  std::vector<Contact> _contacts;
  SUPPORT_MOTION _motion = SUPPORT_MOTION::STOP;
  SUPPORT_STATE _state = SUPPORT_STATE::DEFAULT;
  std::vector<int> _scale = {1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1};
  mwoibn::VectorN _error, _full_state;
};
}
#endif // WHEELED_MOTION_H
