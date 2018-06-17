#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_COM_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_COM_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>

namespace mgnss
{

namespace controllers
{

class WheeledMotionCom : public WheelsControllerExtend
{

public:
WheeledMotionCom(mwoibn::robot_class::Robot& robot);

~WheeledMotionCom() {
}

virtual void updateBase(){

//    std::cout << "_heading\t" << _heading << std::endl;

        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position.head(2));
        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));

}

virtual void fullUpdate(const mwoibn::VectorN& support);
virtual void compute();

virtual double getBaseGroundX()
{
        return _robot.centerOfMass().get()[0];
}
virtual double getBaseGroundY()
{
        return _robot.centerOfMass().get()[1];
}
virtual double getBaseGroundZ()
{
        return _pelvis_position_ptr->points().getPointStateWorld(0)[2];
}
virtual double getBaseGroundRz(){
        return _steering_ptr->getState()[2];
}

protected:

std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass>
_com_ptr;

virtual void _setInitialConditions();
virtual void _createTasks();
virtual void _initIK();

virtual void _correct();

};
}
}

#endif // WHEELED_MOTION_H
