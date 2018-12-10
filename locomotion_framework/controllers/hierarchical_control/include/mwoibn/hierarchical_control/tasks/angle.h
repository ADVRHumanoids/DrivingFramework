#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_ANGLE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_ANGLE_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/angles/basic.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

class Angle : public BasicTask
{

public:
template<typename Other>
  Angle(Other angle, mwoibn::robot_class::Robot& robot)
          : BasicTask()
  {
          _angle_ptr.reset(new Other(angle));
          _init(1, robot.getDofs());
  }

  Angle(Angle&& other)
          : BasicTask(other)
  {
          _angle_ptr = std::move(other._angle_ptr);
          //_init(1, robot.getDofs());
  }

  virtual ~Angle() {
  }

  virtual void updateError();

  double getCurrent();

  void reset();

  virtual void updateJacobian();

  virtual double getReference() const;
  virtual void setReference(double reference);

// virtual double det();

protected:
double _ref;
std::unique_ptr<robot_class::angles::Basic> _angle_ptr;
};

class SoftAngle : public Angle {

public:
  template<typename Other>
    SoftAngle(Other angle, mwoibn::robot_class::Robot& robot)
            : Angle(angle, robot)
    {
            _resteer = false;
    }

virtual ~SoftAngle() {
}

virtual void updateError();
bool resteer();

protected:
bool _resteer;
void _reverse(double limit);
void _saturation(double limit);
void _limit2PI();

};
}
} // namespace package
} // namespace library

#endif
