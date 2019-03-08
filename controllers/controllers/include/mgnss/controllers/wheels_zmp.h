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
#include <mgnss/higher_level/qp/tasks/support_shaping_v4_0.h>
#include <mgnss/higher_level/qp/tasks/support_shaping_v6_0.h>

#include <mgnss/higher_level/qp/tasks/qr_tracking.h>
#include <mgnss/higher_level/qp/tasks/qr_joint_space_v2.h>
#include <mgnss/higher_level/shape_action.h>

#include "mwoibn/robot_points/constant.h"

namespace mgnss
{

namespace controllers {

class WheelsZMP : public WheelsControllerExtend
{

public:
WheelsZMP( mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) : WheelsControllerExtend(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;

        _create(config);
        for(auto& name: _robot.getLinks("wheels"))
          centers__.add(mwoibn::robot_points::LinearPoint(name, _robot));
}



WheelsZMP( mwoibn::robot_class::Robot& robot, YAML::Node config) : WheelsControllerExtend(robot), centers__(_robot.getDofs()), _world(3, _robot.getDofs())
{

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
      _qr_wrappers["SHAPE"]->update();
      _qr_wrappers["SHAPE"]->solve();
      // _qr_wrappers["SHAPE_JOINT"]->update();
      // _qr_wrappers["SHAPE_JOINT"]->solve();

      // std::cout << "get SHAPE\t" << _qr_wrappers["SHAPE"]->get().transpose() << std::endl;
      // std::cout << "raw SHAPE\t" << _qr_wrappers["SHAPE"]->raw().transpose() << std::endl;
      // std::cout << "get SHAPE_JOINT\t" << _qr_wrappers["SHAPE_JOINT"]->get().transpose() << std::endl;
      // std::cout << "raw SHAPE_JOINT\t" << _qr_wrappers["SHAPE_JOINT"]->raw().transpose() << std::endl;
      // std::cout << "fin SHAPE_JOINT\t" << (state_machine__->stateJacobian()*_qr_wrappers["SHAPE_JOINT"]->get() + state_machine__->stateOffset()).transpose() << std::endl;
      // std::cout << "CAMBER error\t" << (_leg_tasks["CAMBER"].first.getError()).transpose() << std::endl;
      // std::cout << "CAMBER solution\t" << (_leg_tasks["CAMBER"].first.getJacobian()*_robot.command.velocity.get()).transpose() << std::endl;
      // std::cout << "CAMBER SHAPE_JOINT\t" << ( _leg_tasks["CAMBER"].first.getJacobian()*_qr_wrappers["SHAPE_JOINT"]->raw()).transpose() << std::endl;
      // std::cout << "STEERING SHAPE_JOINT\t" << ( _leg_tasks["STEERING"].first.getJacobian()*_qr_wrappers["SHAPE_JOINT"]->raw()).transpose() << std::endl;
      // std::cout << "CONTACT_POINTS SHAPE_JOINT\t" << ( _leg_tasks["CONTACT_POINTS"].first.getJacobian()*_qr_wrappers["SHAPE_JOINT"]->raw()).transpose() << std::endl;
      //
      // std::cout << "CAMBER current";
      // for(auto& task: _leg_tasks["CAMBER"].second)
      //   std::cout << "CAMBER current\t" << task.getCurrent();
      // std::cout << std::endl;
      //
      //
      // for(int i = 0; i < 4; i++){
      //   mwoibn::Vector3 temp__;
      //   temp__.setZero();
      //   temp__.head<2>() = (state_machine__->stateJacobian()*_qr_wrappers["SHAPE_JOINT"]->get() + state_machine__->stateOffset()).segment<2>(2*i);
      //   std::cout << i << "\tste SHAPE_JOINT\t" << (state_machine__->steeringFrames()[i]->rotation*(temp__ )).transpose() << std::endl;
      // }

      // for(int i = 0; i < 4; i++)
         // _modified_support.segment<2>(3*i)  = _qr_wrappers["SHAPE"]->get().segment<2>(2*i);
    _modified_support.setZero();
    _support += _modified_support*_robot.rate(); // for this mode integrate current command

    std::cout << "_support\t" << _support.transpose() << std::endl;
    std::cout << "_modified_support\t" << _modified_support.transpose() << std::endl;

    step();

    updateBase();
    _updateSupport();

    _next_step[0] = (_linear_vel[0]);
    _next_step[1] = (_linear_vel[1]);
    _next_step[2] = (_angular_vel[2]); // just limit the difference

    steering();


}


protected:

  mwoibn::VectorN __last_steer;
  // std::unique_ptr<mgnss::higher_level::SupportShapingV4> shape__;

  std::unique_ptr<mgnss::higher_level::StateMachine> state_machine__;
  std::unique_ptr<mwoibn::hierarchical_control::actions::ShapeAction> shape_action__;

  mwoibn::robot_points::Handler<mwoibn::robot_points::LinearPoint> centers__;
  std::unique_ptr<mwoibn::hierarchical_control::tasks::CenterOfMass> _com_ptr;




void _allocate(YAML::Node config);


std::unique_ptr<mwoibn::hierarchical_control::tasks::Aggravated> _contact_point;
mwoibn::VectorN estimated__, _modified_support, _zero;
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
