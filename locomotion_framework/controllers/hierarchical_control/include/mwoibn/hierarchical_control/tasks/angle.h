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
Angle(robot_class::angles::Basic& angle, mwoibn::robot_class::Robot& robot);
virtual ~Angle() {
}

virtual void updateError();

double getCurrent();

virtual void updateJacobian();

virtual double getReference() const;

virtual void setReference(double reference);

protected:
double _ref;
robot_class::angles::Basic& _angle;
};

class SoftAngle : public Angle {

public:
SoftAngle(robot_class::angles::Basic& angle, mwoibn::robot_class::Robot& robot);

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
