#ifndef __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H
#define __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H

#include "mgnss/controllers/wheels_controller.h"
#include <mwoibn/hierarchical_control/tasks/angle.h>
#include <mwoibn/robot_class/angles/camber.h>
#include <mwoibn/robot_class/angles/caster.h>
#include <mwoibn/robot_class/angles/steering.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

namespace mgnss
{

namespace controllers {

class WheelsControllerExtend : public WheelsController
{

public:
WheelsControllerExtend(mwoibn::robot_class::Robot& robot);

virtual ~WheelsControllerExtend() {
}

virtual void compute();
virtual void steering();

virtual void fullUpdate(const mwoibn::VectorN& support);

virtual void resteer(int i)
{
        _resteer[i] = true;
        _start_steer[i] = _test_steer[i];
}
virtual void stopResteer(int i)
{
        _resteer[i] = false;
        _start_steer[i] = _test_steer[i];
}

virtual void resetSteering();

virtual void setSteering(int i, double th)
{
        _leg_tasks["STEERING"].second[i].setReference(th);
}

virtual void setCastor(int i, double th)
{
        _leg_tasks["CASTER"].second[i].setReference(th);
}
virtual void setCamber(int i, double th)
{
        _leg_tasks["CAMBER"].second[i].setReference(th);
}

virtual void setSteering(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _leg_tasks["STEERING"].second[i].setReference(ref[i]);
}

virtual void setCastor(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _leg_tasks["CASTER"].second[i].setReference(ref[i]);
}
virtual void setCamber(const mwoibn::VectorN& ref)
{
        for(int i = 0; i < ref.size(); i++)
                _leg_tasks["CAMBER"].second[i].setReference(ref[i]);
}

double getSteer(int i){
        return _leg_tasks["STEERING"].second[i].getCurrent();
}

const mwoibn::VectorN& errorSteer(){
        return _leg_tasks["STEERING"].first.getError();
}

const mwoibn::VectorInt& countResteer(){
        return _reset_count;
}

virtual void _allocate();


virtual void claim(int i){
        _steering_ptr->claimContact(i);
        _constraints_ptr->claimContact(i);
}

virtual void release(int i){
        _steering_ptr->releaseContact(i);
        _constraints_ptr->releaseContact(i);
}

virtual void log(mwoibn::common::Logger& logger, double time);


protected:
std::map<std::string, std::pair<mwoibn::hierarchical_control::tasks::Aggravated, std::vector<mwoibn::hierarchical_control::tasks::Angle> > > _leg_tasks;
std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _world_posture_ptr;

mwoibn::VectorInt _reset_count;
mwoibn::VectorN _test_steer, _current_steer, _start_steer;
mwoibn::VectorBool _resteer;

void _create(YAML::Node config);
virtual void _correct();

virtual void _createTasks(YAML::Node config);
virtual void _setInitialConditions();
};
}
}
#endif // WHEELED_MOTION_H
