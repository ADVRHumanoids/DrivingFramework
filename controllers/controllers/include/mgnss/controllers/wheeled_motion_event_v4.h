#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_EVENT_4_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_EVENT_4_H

#include "mgnss/controllers/wheels_controller_extend.h"

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionEvent4 : public WheelsControllerExtend
{

public:
WheeledMotionEvent4(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
WheeledMotionEvent4(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~WheeledMotionEvent4() {
}

virtual void log(mwoibn::common::Logger& logger, double time);

void updateBase(){

        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);

        WheelsController::updateBase();
}

const mwoibn::VectorN& getComFull(){
        return _robot.centerOfMass().get();
}
const mwoibn::VectorN& errorCom(){
        return _com_ptr->getError();
}

const mwoibn::VectorN& refCom(){
        return _com_ptr->getReference();
}
double refComX(){
        return _com_ptr->getReference() (0,0);
}
double refComY(){
        return _com_ptr->getReference() (0,1);
}
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

// const mwoibn::VectorN& getSteer(){
//         return _leg_tasks["STEERING"].first.getCurrent();
// }
// double getSteer(int i){
//         return _steer_task[i].getCurrent();
// }
// const mwoibn::VectorN& errorSteer(){
//         return _leg_tasks["STEERING"].first.getError();
// }

const mwoibn::VectorBool& isResteer(){
        return _resteer;
}
// const mwoibn::VectorN& getAnkleYaw(){
//         return _test_steer;
// }
//  mwoibn::VectorN getBase(){return _robot.state.position.get().head<3>();}
const mwoibn::VectorN& getBaseError(){
        return _pelvis_position_ptr->getError();
}
// const mwoibn::VectorN& getBaseOrnError(){
//         return _pelvis_orientation_ptr->getError();
// }
// const mwoibn::VectorInt& countResteer(){
//         return _reset_count;
// }


protected:
void _allocate(YAML::Node config);

std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;

mwoibn::VectorN _com_ref;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
