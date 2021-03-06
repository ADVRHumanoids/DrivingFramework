#ifndef __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_H
#define __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_H

#include "mgnss/modules/base.h"
#include "mgnss/controllers/ik_base.h"

#include <mwoibn/hierarchical_control/tasks/constraints_task.h>

#include <mwoibn/hierarchical_control/tasks/contact_point.h>
#include "mgnss/higher_level/steering_reference.h"

#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

namespace mgnss
{

namespace controllers {

class WheelsController : public IKBase
{

public:
WheelsController(mwoibn::robot_class::Robot& robot);

virtual ~WheelsController() {
}


virtual void update(){
        nextStep();
        compute();
}

void compute();


virtual void setRate(){
        _dt = _robot.rate(); _steering_ref_ptr->setRate(_dt);
}

virtual void resetSteering() = 0;

virtual void setSteering(int i, double th) = 0;

void rotateBaseX(double th)
{
        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, th)*(_orientation);
}

void rotateBaseY(double th)
{
        _orientation = mwoibn::Quaternion::fromAxisAngle(_y, th)*(_orientation);
}

virtual void setBaseRotVelX(double dth) {
        _angular_vel[0] = dth;
}
virtual void setBaseRotVelY(double dth) {
        _angular_vel[1] = dth;
}

void setBaseX(double x){
        _position[0] = x;
}
void setBaseY(double y){
        _position[1] = y;
}
void setBaseZ(double z){
        _position[2] = z;
}

virtual void setBaseHeading(double th) {
        _heading = th;
}

virtual void setBaseDotX(double dx) {
        _linear_vel[0] = dx;
}
virtual void setBaseDotY(double dy) {
        _linear_vel[1] = dy;
}
virtual void setBaseDotZ(double dz) {
        _linear_vel[2] = dz;
}
virtual void setBaseDotHeading(double th) {
        _angular_vel[2] = th;
}

mwoibn::VectorN getSteering() {
        return steerings;
}

virtual const mwoibn::Vector3& getCp(int i){
        return _steering_ptr->getPointStateReference(i);
}
virtual mwoibn::VectorN errorCp(int i){
        return _steering_ptr->getReferenceError(i);
}
virtual const mwoibn::VectorN& refCp(){
        return _steering_ptr->getReference();
}

virtual const mwoibn::VectorN& refSteer(){
        return steerings;
}

virtual const mwoibn::Vector3& getLinVel(){
        return _linear_vel;
}
virtual const mwoibn::Vector3& getAngVel(){
        return _angular_vel;
}

void setSupport(const mwoibn::VectorN& support){
        _support.noalias() = support;
}

void setSupport(int i, double pos){_support[i] = pos;}

void setSupportVel(const mwoibn::VectorN& support_vel){
        _support_vel.noalias() = support_vel;
}

void setSupportVel(int i, double vel){_support_vel[i] = vel;}


virtual double getBaseGroundX() = 0;
virtual double getBaseGroundY() = 0;
virtual double getBaseGroundZ() = 0;


virtual void updateBase(){
          _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
          _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));
}

virtual void step();

virtual void steering();


virtual double limit(const double th);


virtual const mwoibn::VectorN& getSupportReference()
{
        return _steering_ptr->getReference();
}
virtual const mwoibn::VectorN& getBodyPosition()
{
        return _pelvis_position_ptr->getReference();
}

virtual void nextStep();


const mgnss::higher_level::SteeringReference& steering_task(){return *_steering_ref_ptr;}

protected:

virtual void _updateSupport()
{
        _steering_ptr->setReference(_support);
}

std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints> _constraints_ptr;

std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianSelective> _pelvis_position_ptr;
std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective> _pelvis_orientation_ptr;

std::unique_ptr<mwoibn::hierarchical_control::tasks::ContactPoint> _steering_ptr;

std::unique_ptr<mgnss::higher_level::SteeringReference> _steering_ref_ptr;



//double rate = 200;
double orientation = 0, _heading;
mwoibn::VectorN steerings, _previous_command, _support, _support_vel;
mwoibn::Vector3 _position, _next_step, _angular_vel, _linear_vel;

mwoibn::Quaternion _orientation;
bool _reference = false;
mwoibn::VectorInt _select_steer;
mwoibn::VectorN _l_limits, _u_limits;
int count = 0;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);

// virtual double _readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain);
//virtual void _initSteering(YAML::Node config, std::function<>);


};
}
}
#endif // WHEELED_MOTION_H
