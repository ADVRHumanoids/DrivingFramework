#ifndef PROGRAMS_WHEELED_MOTION_H
#define PROGRAMS_WHEELED_MOTION_H

//#include <mwoibn/hierarchical_control/tasks/cartesian_simplified_pelvis_task_v3.h>
#include "mgnss/controllers/wheels_controller.h"

namespace mgnss
{

namespace controllers
{

class WheeledMotion : public mgnss::controllers::WheelsController
{

public:
WheeledMotion(mwoibn::robot_class::Robot& robot);

virtual ~WheeledMotion() {
}

virtual void resetSteering();

virtual void updateBase()
{
        _pelvis_position_ptr->setReference(0, _position);

        _pelvis_orientation_ptr->setReference(
                0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*_pelvis_orientation_ptr->getOffset(0));
}

virtual void step(){
        for (int i = 0; i < _position.size(); i++)
        {
                if (_previous_command[i] != _linear_vel[i])
                        _position[i] = _pelvis_position_ptr->points().getPointStateWorld(0)[i];
                _previous_command[i] = _linear_vel[i];
        }

        WheelsController::step();
}


virtual void steering();

virtual void fullUpdate(const mwoibn::VectorN& support);
virtual void fullUpdate(const mwoibn::VectorN& support,
                        const mwoibn::Vector3& velocity, const double omega);

using WheelsController::update;
virtual void update(const mwoibn::VectorN& support,
                    const mwoibn::Vector3& velocity, const double omega);

virtual bool isDoneSteering(const double eps) const
{
        return _isDone(*_leg_z_ptr, eps);
}
virtual bool isDonePlanar(const double eps) const
{
        return _isDone(*_steering_ptr, eps);
}
virtual bool isDoneWheels(const double eps) const
{
        return _isDone(*_leg_xy_ptr, eps);
}

virtual double getBaseGroundX()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[0];
}
virtual double getBaseGroundY()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[1];
}
virtual double getBaseGroundZ()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[2];
}

virtual double getBaseGroundRz(){
        return _steering_ptr->getState()[2];
}

protected:
std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective>
_leg_xy_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective>
_leg_z_ptr;

virtual void _setInitialConditions();
virtual void _createTasks();
virtual void _initIK();

};
}
}
#endif // WHEELED_MOTION_H
