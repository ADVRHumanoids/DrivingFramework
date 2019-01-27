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

#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_tracking_task.h>
//#include "mgnss/higher_level/steering_reactif_zmp.h"

#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

#include <mwoibn/robot_points/linear_point.h>

//TEMP
#include <mwoibn/dynamic_models/basic_model.h>

#include <mgnss/higher_level/state_machine.h>
#include <mgnss/higher_level/support_shaping_v3_0.h>
#include <mgnss/higher_level/qr_tracking.h>

#include "mwoibn/robot_points/constant.h"

namespace mgnss
{

namespace controllers {

class WheelsZMP : public WheelsControllerExtend
{

public:
WheelsZMP( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot), __dynamics(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;
        // shape__.reset(new mgnss::higher_level::SupportShapingV3(robot, config));

        _create(config);
        for(auto& name: _robot.getLinks("wheels"))
          centers__.add(mwoibn::robot_points::LinearPoint(name, _robot));
}



WheelsZMP( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot), __dynamics(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{
        // shape__.reset(new mgnss::higher_level::SupportShapingV3(robot, config));

        _create(config);
        for(auto& name: _robot.getLinks("wheels"))
          centers__.add(mwoibn::robot_points::LinearPoint(name, _robot));
}

virtual ~WheelsZMP() {
}

void compute();
void steering();

void step();


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
//
// virtual const mwoibn::Vector3& getCp(int i){
//         return _steering_ptr_2->getPointStateReference(i);
// }
// virtual mwoibn::VectorN errorCp(int i){
//         return _steering_ptr_2->getReferenceError(i);
// }
// virtual const mwoibn::VectorN& refCp(){
//         return _steering_ptr_2->getReference();
// }

virtual const mwoibn::VectorN& refSteer(){
        return steerings;
}

//virtual void step();

// const mwoibn::VectorN& getSupportReference()
// {
//         return _steering_ptr_2->getReference();
// }
const mwoibn::VectorN& getBodyPosition()
{
        return _pelvis_position_ptr->getReference();
}

virtual void nextStep(){

    _robot.centerOfMass().update();

    _support += _support_vel*_robot.rate();
    state_machine__->update();
    // if(state_machine__->restart()){
    // restore a desired postion from a current one
      // std::cout << "restart" << std::endl;
    // for(int i = 0; i < 4; i++)
    //   _modified_support.segment<3>(3*i) = _steering_ptr->getPointStateReference(i);
    // }
    // if(state_machine__->state()){
    //
    //   // std::cout << "nextStep::restore" << std::endl;
    //   restore__->solve();
    // }
    // else{
      // std::cout << "nextStep::shape" << std::endl;
      shape__->solve();
    // }

    // for(int i = 0; i < 4; i++)
    //    _support_vel.segment<2>(3*i) = restore__->get().segment<2>(2*i);

    // std::cout << "restore\t" << restore__->get().transpose() << std::endl;
    // std::cout << "_support\t" << _support.transpose() << std::endl;
    // std::cout << "_modified_support\t" << _modified_support.transpose() << std::endl;

    step();

    updateBase();
    _updateSupport();

    _next_step[0] = (_linear_vel[0]);
    _next_step[1] = (_linear_vel[1]);
    _next_step[2] = (_angular_vel[2]); // just limit the difference

    steering();

   //  if (state_machine__->state()){
   //     for(int i = 0; i < 4; i++)
   //        _modified_support.segment<2>(3*i)  -= restore__->get().segment<2>(2*i) * _robot.rate();
   //      }
   // else {
   //     for(int i = 0; i < 4; i++)
   //        _modified_support.segment<2>(3*i)  -= shape__->get().segment<2>(2*i) * _robot.rate();
   //      }
   //
   //  mwoibn::Vector3 temp_state__ = mwoibn::Vector3::Zero();
   //  for(int i = 0; i < 4; i++){
   //    temp_state__.setZero();
   //    if (state_machine__->state())
   //      temp_state__.head<2>() = restore__->get().segment<2>(2*i);
   //    else
   //      temp_state__.head<2>() = shape__->get().segment<2>(2*i);
   //
   //      mwoibn::Matrix3 rot;
   //      rot << std::cos(steerings[i]), -std::sin(steerings[i]), 0, std::sin(steerings[i]), std::cos(steerings[i]), 0, 0, 0, 1;
   //    temp_state__ = rot.transpose()*temp_state__; // this should be in a steering frames
   //    temp_state__.tail<2>().setZero(); // remove other components

   //    // Apply in a world frame - I am ignoring heading here!
   //    _modified_support.segment<2>(3*i)  += (rot*temp_state__).head<2>() * _robot.rate();
      // std::cout << "temp_state__\t" << temp_state__.transpose() << std::endl;
      // std::cout << "modification\t" <<  ((rot*temp_state__).head<2>() * _robot.rate()).transpose() << std::endl;
      // std::cout << "restore__ command\t" <<  (restore__->get().segment<2>(2*i) * _robot.rate()).transpose() << std::endl;
      // std::cout << "shape__ command\t" <<  (shape__->get().segment<2>(2*i) * _robot.rate()).transpose() << std::endl;
     // }

    // _updateSupport();

}


protected:

  mwoibn::dynamic_models::BasicModel __dynamics;
  std::unique_ptr<mgnss::higher_level::SupportShapingV3> shape__;
  std::unique_ptr<mgnss::higher_level::StateMachine> state_machine__;
  std::unique_ptr<mgnss::higher_level::QrTracking> restore__;


  mwoibn::robot_points::Handler<mwoibn::robot_points::LinearPoint> centers__;

// virtual void _updateSupport()
// {
//   for(int i = 0, k = 0; i < _steering_select.size(); i++){
//         _steering_select[i] ? _steering_ptr_2->setReference(k, _support.segment<3>(3*i)) : _steering_ptr->setReference(i-k, _support.segment<3>(3*i));
//         k += _steering_select[i];
//   }
// }


// std::unique_ptr<mwoibn::hierarchical_control::tasks::ContactPointZMPV2> _steering_ptr_2;
std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;

// mwoibn::VectorBool _steering_select;


void _allocate(YAML::Node config);


std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _contact_point;
mwoibn::VectorN estimated__, _modified_support;
mwoibn::VectorN _com_ref;

mwoibn::robot_points::Constant _world;

virtual void _setInitialConditions();
virtual void _allocate();
virtual void _createTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);

virtual void _updateSupport()
{
        _steering_ptr->setReference(_support);
}

};
}
}

#endif // WHEELED_MOTION_H
