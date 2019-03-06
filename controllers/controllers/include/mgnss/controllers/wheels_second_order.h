#ifndef __MGNSS_CONTROLLERS_WHEELS_SECOND_ORDER_H
#define __MGNSS_CONTROLLERS_WHEELS_SECOND_ORDER_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

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
#include <mwoibn/hierarchical_control/tasks/angle.h>
#include <mwoibn/robot_class/angles/camber.h>
#include <mwoibn/robot_class/angles/caster.h>
#include <mwoibn/robot_class/angles/steering.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

#include "mgnss/modules/base.h"

#include <mwoibn/hierarchical_control/controllers/basic.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_second_order.h>
#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mgnss/higher_level/state_machine.h>

//#include "mgnss/higher_level/steering_reactif_zmp.h"

#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>
#include <mgnss/higher_level/qp_action.h>

#include <mwoibn/robot_points/linear_point.h>

//TEMP


#include <mgnss/higher_level/qr_task_wrapper.h>
#include <mgnss/higher_level/qp_aggravated.h>
#include <mgnss/higher_level/support_shaping_v4_0.h>

#include "mwoibn/robot_points/constant.h"

namespace mgnss
{

namespace controllers {

class WheelsSecondOrder : public mgnss::modules::Base
{

public:
WheelsSecondOrder( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : mgnss::modules::Base(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{

        _ik_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));


        _x << 1, 0, 0;
        _y << 0, 1, 0;
        _z << 0, 0, 1;

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;

        _create(config);
        for(auto& name: _robot.getLinks("wheels"))
          centers__.add(mwoibn::robot_points::LinearPoint(name, _robot));
}



WheelsSecondOrder( mwoibn::robot_class::Robot& robot, YAML::Node config) : mgnss::modules::Base(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{
  _ik_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));


  _x << 1, 0, 0;
  _y << 0, 1, 0;
  _z << 0, 0, 1;

        _create(config);
        for(auto& name: _robot.getLinks("wheels"))
          centers__.add(mwoibn::robot_points::LinearPoint(name, _robot));
}

virtual ~WheelsSecondOrder() {
}


virtual void init(){
        _robot.wait();
        _robot.get();
        _robot.updateKinematics();
        _robot.centerOfMass().update();

        _setInitialConditions();
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

void compute();
void steering();

virtual void fullUpdate(const mwoibn::VectorN& support);

virtual void setSteering(int i, double th)
{
}

virtual void setCastor(int i, double th)
{
        _leg_tasks["CASTER"].second[i].setReference(th);
}
virtual void setCamber(int i, double th)
{
        _leg_tasks["CAMBER"].second[i].setReference(th);
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


void step();


void claim(int i){
        _constraints_ptr->claimContact(i);
}

void release(int i){
        _constraints_ptr->releaseContact(i);
}

virtual void log(mwoibn::common::Logger& logger, double time);

void updateBase(){
        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);
        _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*(mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate()))*(_orientation);
        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));
}

mwoibn::VectorN getBaseReference(){
        return _pelvis_position_ptr->getReference(0);
}

mwoibn::VectorN getBase(){
        return _pelvis_position_ptr->points().getPointStateWorld(0);
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

virtual void setRate(){
     // _steering_ref_ptr->setRate(_robot.rate());
}

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

virtual const mwoibn::Vector3& getCp(int i){
        return _steering_ptr->getPointStateReference(i);
}
virtual mwoibn::VectorN errorCp(int i){
        return _steering_ptr->getReferenceError(i);
}
virtual const mwoibn::VectorN& refCp(){
        return _steering_ptr->getReference();
}


// virtual const mwoibn::VectorN& refSteer(){
//         return steerings;
// }

virtual const mwoibn::Vector3& getLinVel(){
        return _linear_vel;
}
virtual const mwoibn::Vector3& getAngVel(){
        return _angular_vel;
}

virtual const mwoibn::VectorN& getSupportReference()
{
        return _steering_ptr->getReference();
}
const mwoibn::VectorN& getBodyPosition()
{
        return _pelvis_position_ptr->getReference();
}


void setSupport(const mwoibn::VectorN& support){
        _support.noalias() = support;
}

void setSupport(int i, double pos){_support[i] = pos;}

void setSupportVel(const mwoibn::VectorN& support_vel){
        _support_vel.noalias() = support_vel;
}

void setSupportVel(int i, double vel){_support_vel[i] = vel;}


virtual void nextStep();


virtual double limit(const double th);

virtual bool isRunning() {
        return _robot.isRunning();
}

// const mgnss::higher_level::SteeringReference& steering_task(){return *_steering_ref_ptr;}


protected:


  std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints> _constraints_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianSelective> _pelvis_position_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective> _pelvis_orientation_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::tasks::ContactPointSecondOrder> _steering_ptr;
  // std::unique_ptr<mgnss::higher_level::SupportShapingV4> shape__;

  std::map<std::string, std::unique_ptr<mgnss::higher_level::QrTask> > _qr_wrappers;
  std::vector<std::unique_ptr<mgnss::higher_level::QpAggravated> > _qp_aggravated;
  // std::unique_ptr<mgnss::higher_level::SteeringReference> _steering_ref_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::controllers::Actions> _ik_ptr;
  std::unique_ptr<mgnss::higher_level::StateMachine> state_machine__;

  std::map<std::string, mwoibn::hierarchical_control::tasks::BasicTask*> _tasks;  // Adding that only helps with automatic IK generation
  std::map<std::string, std::shared_ptr<mwoibn::hierarchical_control::actions::Task> > _actions;  // Adding that only helps with automatic IK generation

  // std::unique_ptr<mgnss::higher_level::QRJointSpaceV2> shape_joint__;
  // std::unique_ptr<mgnss::higher_level::QRJointSpaceV2> shape_wheel__;



  double rate = 200;
  double _dt, orientation = 0, _heading;
  mwoibn::VectorN _command, _previous_command, _support, _support_vel;
  mwoibn::Vector3 _position, _next_step, _angular_vel, _linear_vel;
  mwoibn::Axis _x, _y, _z;

  mwoibn::Quaternion _orientation;
  bool _reference = false;
  mwoibn::VectorInt _select_steer, _select_ik;
  mwoibn::VectorN _l_limits, _u_limits;
  int count = 0;
  std::vector<std::string> _log_names;

  mwoibn::VectorN __last_steer;

  // std::unique_ptr<mgnss::higher_level::QrTaskWrapper> qr_tracking;
  // std::unique_ptr<mwoibn::hierarchical_control::actions::QP> qp_action;


  mwoibn::robot_points::Handler<mwoibn::robot_points::LinearPoint> centers__;

std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;

std::map<std::string, std::pair<mwoibn::hierarchical_control::tasks::Aggravated, std::vector<mwoibn::hierarchical_control::tasks::Angle> > > _leg_tasks;
std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _world_posture_ptr;

// mwoibn::VectorInt _reset_count;
// mwoibn::VectorN _test_steer, _current_steer, _start_steer;
// mwoibn::VectorBool _resteer;

void _allocate(YAML::Node config);


std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _contact_point;
mwoibn::VectorN estimated__, _modified_support, _zero;
mwoibn::VectorN _com_ref;

mwoibn::robot_points::Constant _world;
std::unique_ptr<mwoibn::robot_points::LinearPoint> _pelvis;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);
virtual mwoibn::hierarchical_control::actions::Task& _createAction(std::string task, YAML::Node config, YAML::Node full_config);
virtual std::shared_ptr<mwoibn::hierarchical_control::actions::Task> _taskAction(std::string task, YAML::Node config, std::string type, YAML::Node full_config);
double _readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain );

void _create(YAML::Node config);

virtual void _updateSupport()
{
        _steering_ptr->setReference(_support);
}

};
}
}

#endif // WHEELED_MOTION_H
