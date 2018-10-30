#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_ACTIONS_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_ACTIONS_H

#include "mgnss/controllers/wheels_controller_extend.h"

#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/controllers/actions.h>

// #include <mwoibn/hierarchical_control/actions/merge.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionActions : public WheelsControllerExtend
{

public:
WheeledMotionActions(mwoibn::robot_class::Robot& robot, std::string config_file);
WheeledMotionActions(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~WheeledMotionActions() {
}

virtual void init();

virtual void startLog(mwoibn::common::Logger& logger);
virtual void log(mwoibn::common::Logger& logger, double time);

void resteer(int i){
        _resteer[i] = true;
        _start_steer[i] = _test_steer[i];
}

void stopResteer(int i){
        _resteer[i] = false;
        _start_steer[i] = _test_steer[i];
}

void updateBase(){

        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);

        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));


//        std::cout << std::fixed;
//        std::cout << std::setprecision(8);
/*
        std::cout << "camber\t";
        for(auto& camber: _camber_task)
          std::cout << camber.getError() << "\t";

        std::cout << "caster\t";
        for(auto& caster: _caster_task)
            std::cout << caster.getError() << "\t";

        std::cout << std::endl;
        */
}

void steering();

void fullUpdate(const mwoibn::VectorN& support);
void compute();

virtual void switchToCastor(double mu){
        std::cout << "switchToCastor" << std::endl;
        _actions_ptr->replace(_leg_castor, mu);
}

virtual void switchToCamber(double mu){
        std::cout << "switchToCamber" << std::endl;
        _actions_ptr->replace(_leg_camber, mu);
}


mwoibn::VectorN getCom(){
        return _robot.centerOfMass().get().head<2>();
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
virtual double getBaseGroundRz(){
        return _steering_ptr->getState()[2];
}
double getSteer(int i){
        return _steer_task[i].getCurrent();
}
const mwoibn::VectorN& errorSteer(){
        return _leg_steer.getError();
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
//  mwoibn::VectorN getBase(){return _robot.state.position.get().head<3>();}
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
WheeledMotionActions(mwoibn::robot_class::Robot& robot) : WheelsControllerExtend(robot){
}

void _allocate(YAML::Node config);

std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _world_posture_ptr;

mwoibn::hierarchical_control::controllers::Actions* _actions_ptr;
mwoibn::VectorInt _select_wheel, _reset_count;
mwoibn::VectorN _test_steer, _current_steer, _start_steer;
mwoibn::VectorBool _resteer;

mwoibn::VectorN _com_ref;

virtual void _correct();
void _createAngleTasks(YAML::Node config);
virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
