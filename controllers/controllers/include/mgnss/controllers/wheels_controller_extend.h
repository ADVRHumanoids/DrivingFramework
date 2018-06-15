#ifndef __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H
#define __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H

#include <mgnss/controllers/wheels_controller.h>
#include "mwoibn/hierarchical_control/tasks/angle.h"
#include "mwoibn/robot_class/angles/camber.h"
#include "mwoibn/robot_class/angles/caster.h"
#include "mwoibn/robot_class/angles/steering.h"
#include "mwoibn/hierarchical_control/tasks/aggravated.h"
//#include <mwoibn/hierarchical_control/tasks/castor_angle_task.h>
//#include <mwoibn/hierarchical_control/tasks/camber_angle_task_2.h>
//#include <mwoibn/hierarchical_control/tasks/steering_angle_task.h>

namespace mgnss
{

namespace controllers {

class WheelsControllerExtend : public WheelsController
{

public:
WheelsControllerExtend(mwoibn::robot_class::Robot& robot);

~WheelsControllerExtend() {
}

void resetSteering();

void setSteering(int i, double th)
{
        _steer_task[i].setReference(th);
}

void setCastor(int i, double th)
{
        _caster_task[i].setReference(th);
}
void setCamber(int i, double th)
{
        _camber_task[i].setReference(th);
}

void setSteering(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _steer_task[i].setReference(ref[i]);
}

void setCastor(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _caster_task[i].setReference(ref[i]);
}
void setCamber(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _camber_task[i].setReference(ref[i]);
}

virtual bool isDoneSteering(const double eps) const
{
        return _isDone(_leg_steer, eps);
}
virtual bool isDonePlanar(const double eps) const
{
        return _isDone(*_steering_ptr, eps);
}
virtual bool isDoneWheels(const double eps) const
{
        return _isDone(_leg_camber, eps);
}

void claim(int i){
        _steering_ptr->claimContact(i);
        _constraints_ptr->claimContact(i);
}

void release(int i){
        _steering_ptr->releaseContact(i);
        _constraints_ptr->releaseContact(i);
}

protected:

std::vector<mwoibn::robot_class::angles::Caster> _caster;
std::vector<mwoibn::robot_class::angles::Steering> _steer;
std::vector<mwoibn::robot_class::angles::Camber> _camber;

std::vector<mwoibn::hierarchical_control::tasks::Angle> _caster_task;
std::vector<mwoibn::hierarchical_control::tasks::SoftAngle> _steer_task;
std::vector<mwoibn::hierarchical_control::tasks::Angle> _camber_task;

mwoibn::hierarchical_control::tasks::Aggravated _leg_camber;
mwoibn::hierarchical_control::tasks::Aggravated _leg_castor;
mwoibn::hierarchical_control::tasks::Aggravated _leg_steer;

void _createAngleTasks();
void _setInitialConditions();

};
}
}
#endif // WHEELED_MOTION_H
