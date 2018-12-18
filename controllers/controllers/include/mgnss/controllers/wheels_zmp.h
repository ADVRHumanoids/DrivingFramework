#ifndef __MGNSS_CONTROLLERS_WHEELS_ZMP_H
#define __MGNSS_CONTROLLERS_WHEELS_ZMP_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

#include "mgnss/controllers/wheels_controller.h"
#include <mwoibn/hierarchical_control/tasks/angle.h>
#include <mwoibn/robot_class/angles/camber.h>
#include <mwoibn/robot_class/angles/caster.h>
#include <mwoibn/robot_class/angles/steering.h>
#include <mwoibn/hierarchical_control/tasks/aggravated.h>

#include "mgnss/modules/base.h"

#include <mwoibn/hierarchical_control/controllers/basic.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>

#include <mgnss/controllers/devel/contact_point_zmp.h>

//#include <mwoibn/hierarchical_control/tasks/contact_point_zmp.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_tracking_task.h>
//#include "mgnss/higher_level/steering_reactif_zmp.h"

#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

#include <mwoibn/robot_points/linear_point.h>
namespace mgnss
{

namespace controllers {

class WheelsZMP : public WheelsControllerExtend
{

public:
WheelsZMP( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;
        _create(config);
}



WheelsZMP( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot)
{
        _create(config);
}

virtual ~WheelsZMP() {
}

void compute();
void steering();

void claim(int i){
        //_steering_ptr_2->claimContact(i);
        _constraints_ptr->claimContact(i);
}

void release(int i){
        //_steering_ptr_2->releaseContact(i);
        _constraints_ptr->releaseContact(i);
}

virtual void log(mwoibn::common::Logger& logger, double time);


void updateBase(){
        _com_ref << _position[0], _position[1];
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_com_ref);
        WheelsController::updateBase();
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
     _steering_ref_ptr->setRate(_robot.rate());
}

virtual void setBaseRotVelX(double dth) {
        _angular_vel[0] = dth;
}
virtual void setBaseRotVelY(double dth) {
        _angular_vel[1] = dth;
}

virtual void setBaseHeading(double th) {
        _heading = th;
}

virtual const mwoibn::Vector3& getCp(int i){
        return _steering_ptr_2->getPointStateReference(i);
}
virtual mwoibn::VectorN errorCp(int i){
        return _steering_ptr_2->getReferenceError(i);
}
virtual const mwoibn::VectorN& refCp(){
        return _steering_ptr_2->getReference();
}

virtual const mwoibn::VectorN& refSteer(){
        return steerings;
}

//virtual void step();

const mwoibn::VectorN& getSupportReference()
{
        return _steering_ptr_2->getReference();
}
const mwoibn::VectorN& getBodyPosition()
{
        return _pelvis_position_ptr->getReference();
}

//virtual void nextStep();


protected:

virtual void _updateSupport()
{
  for(int i = 0, k = 0; i < _steering_select.size(); i++){
        _steering_select[i] ? _steering_ptr_2->setReference(k, _support.segment<3>(3*i)) : _steering_ptr->setReference(i-k, _support.segment<3>(3*i));
        k += _steering_select[i];
  }
}


std::unique_ptr<mwoibn::hierarchical_control::tasks::ContactPointZMP> _steering_ptr_2;
std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;

mwoibn::VectorBool _steering_select;


void _allocate(YAML::Node config);


std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _contact_point;

mwoibn::VectorN _com_ref;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
