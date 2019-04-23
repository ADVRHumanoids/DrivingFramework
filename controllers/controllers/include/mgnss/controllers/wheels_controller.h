#ifndef __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_H
#define __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_H

#include "mgnss/modules/base.h"

#include <mwoibn/hierarchical_control/controllers/basic.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>

#include <mwoibn/hierarchical_control/tasks/contact_point.h>
#include "mgnss/higher_level/steering_reference.h"

#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

#include <mwoibn/hierarchical_control/controllers/actions.h>
#include <mwoibn/hierarchical_control/actions/task.h>
#include <mwoibn/hierarchical_control/actions/compute.h>

#include <mgnss/higher_level/qp/qp_action.h>
#include <mgnss/higher_level/qp/tasks/qp_aggravated.h>

#include <mgnss/higher_level/qp/tasks/qr_task_wrapper.h>

#include <chrono>
namespace mgnss
{

namespace controllers {

class WheelsController : public mgnss::modules::Base
{

public:
WheelsController(mwoibn::robot_class::Robot& robot);

virtual ~WheelsController() {
}

virtual void init(){
        _robot.wait();
        _robot.get();
        _robot.updateKinematics();
        _robot.centerOfMass().update();

        _setInitialConditions();
}


virtual void log(mwoibn::common::Logger& logger, double time){
}

virtual void stop(){
        _command.setZero();
        _robot.command.velocity.set(_command);
        _robot.send();
}

virtual void send(){
        _robot.send();
}

virtual void close(){}

virtual void setRate(double rate){
        mgnss::modules::Base::setRate(rate);
        setRate();
}

virtual void update(){
        nextStep();
        compute();
}

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

virtual void compute();

virtual double limit(const double th);

virtual bool isRunning() {
        return _robot.isRunning();
}


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
// std::vector<std::unique_ptr<mgnss::higher_level::QrTaskWrapper> > _qr_wrappers;
std::map<std::string, std::unique_ptr<mgnss::higher_level::QrTask> > _qr_wrappers;
std::vector<std::unique_ptr<mgnss::higher_level::QpAggravated> > _qp_aggravated;


std::unique_ptr<mwoibn::hierarchical_control::controllers::Actions> _ik_ptr;

std::map<std::string, mwoibn::hierarchical_control::tasks::BasicTask*> _tasks;  // Adding that only helps with automatic IK generation
std::map<std::string, std::shared_ptr<mwoibn::hierarchical_control::actions::Task> > _actions;  // Adding that only helps with automatic IK generation


double rate = 200;
double _dt, orientation = 0, _heading;
mwoibn::VectorN steerings, _command, _previous_command, _support, _support_vel;
mwoibn::Vector3 _position, _next_step, _angular_vel, _linear_vel;
mwoibn::Axis _x, _y, _z;

mwoibn::Quaternion _orientation;
bool _reference = false;
mwoibn::VectorInt _select_steer, _select_ik;
mwoibn::VectorN _l_limits, _u_limits;
mwoibn::VectorN _active_state;
int count = 0;
std::vector<std::string> _log_names;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual mwoibn::hierarchical_control::actions::Task& _createAction(std::string task, YAML::Node config, YAML::Node full_config);

virtual std::shared_ptr<mwoibn::hierarchical_control::actions::Task> _taskAction(std::string task, YAML::Node config, std::string type, YAML::Node full_config);
virtual double _readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain);

virtual void _initIK(YAML::Node config);
//virtual void _initSteering(YAML::Node config, std::function<>);


};
}
}
#endif // WHEELED_MOTION_H
