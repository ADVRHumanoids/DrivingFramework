#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_FULL_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_FULL_H

#include "mgnss/controllers/wheels_controller_extend.h"

namespace mgnss
{

namespace controllers {

class WheeledMotionFull : public WheelsControllerExtend
{

public:
WheeledMotionFull(mwoibn::robot_class::Robot& robot);

~WheeledMotionFull() {
}


void updateBase(){

        _pelvis_position_ptr->setReference(0, _position);

        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));

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

void fullUpdate(const mwoibn::VectorN& support);

protected:

mwoibn::VectorN _test_limits;

virtual void _setInitialConditions();
virtual void _createTasks();
virtual void _initIK();

};
}
}

#endif // WHEELED_MOTION_H
