#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_EVENT_3_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_EVENT_3_H

#include <mgnss/controllers/wheels_controller_extend.h>

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionEvent3 : public WheelsControllerExtend
{

public:
WheeledMotionEvent3(mwoibn::robot_class::Robot& robot, std::string config_file);
WheeledMotionEvent3(mwoibn::robot_class::Robot& robot, YAML::Node config);

~WheeledMotionEvent3() {
}

virtual void init();

virtual void startLog(mwoibn::common::Logger& logger);
virtual void log(mwoibn::common::Logger& logger, double time);

void resteer(int i){
        _resteer[i] = true;
        _start_steer[i] = _test_steer[i];

        //std::cout << "started resteering" << std::endl;
}
void stopResteer(int i){
        _resteer[i] = false;
        _start_steer[i] = _test_steer[i];

        //std::cout << "stoped resteering" << std::endl;
}



void updateBase(){

        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);

        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));
}

void steering();

void fullUpdate(const mwoibn::VectorN& support);
void compute();

mwoibn::VectorN getCom(){
        return _robot.centerOfMass().get().head<2>();
}
const mwoibn::Vector3& getComFull(){
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
virtual double getBaseGroundRz(){
        return _steering_ptr->getState()[2];
}
const mwoibn::VectorN& getSteer(){
        return _leg_steer_ptr->getCurrent();
}
const mwoibn::VectorN& errorSteer(){
        return _leg_steer_ptr->getError();
}
const mwoibn::VectorN& getSteerICM(){
        return _steering_ref_ptr->getICM();
}
const mwoibn::VectorN& getSteerSP(){
        return _steering_ref_ptr->getSP();
}
const mwoibn::VectorN& getVelICM(){
        return _steering_ref_ptr->vICM();
}
const mwoibn::VectorN& getVelSP(){
        return _steering_ref_ptr->vSP();
}
const mwoibn::VectorN& getDampingICM(){
        return _steering_ref_ptr->getDampingICM();
}
const mwoibn::VectorN& getDampingSP(){
        return _steering_ref_ptr->getDampingSP();
}
const mwoibn::VectorBool& isResteer(){
        return _resteer;
}
const mwoibn::VectorN& getAnkleYaw(){
        return _test_steer;
}
//  mwoibn::VectorN getBase(){return _robot.state.get().head<3>();}
const mwoibn::VectorN& getBaseError(){
        return _pelvis_position_ptr->getError();
}
const mwoibn::VectorN& getBaseOrnError(){
        return _pelvis_orientation_ptr->getError();
}
const mwoibn::VectorInt& countResteer(){
        return _reset_count;
}
const mwoibn::VectorN& getVel(){
        return _steering_ref_ptr->v();
}
const mwoibn::VectorN& getDamp(){
        return _steering_ref_ptr->damp();
}
const mwoibn::VectorN& rawSteer(){
        return _steering_ref_ptr->getLimited();
}
const mwoibn::VectorN& pureSteer(){
        return _steering_ref_ptr->getRaw();
}


protected:
void _allocate(YAML::Node config);

std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _world_posture_ptr;


mwoibn::VectorInt _select_wheel, _reset_count;
mwoibn::VectorN _test_steer, _current_steer, _start_steer;
mwoibn::VectorBool _resteer;

mwoibn::VectorN _com_ref;

virtual void _correct();

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks();
virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
