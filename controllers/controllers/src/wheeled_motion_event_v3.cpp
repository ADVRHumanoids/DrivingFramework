#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/higher_level/steering_v8.h"
#include "mgnss/higher_level/steering_reactif.h"
#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>


void mgnss::controllers::WheeledMotionEvent3::_allocate(){

        WheelsControllerExtend::_allocate();

        _com_ref.setZero(2);

}

void mgnss::controllers::WheeledMotionEvent3::_initIK(YAML::Node config){

        WheelsController::_initIK(config);

        YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

        _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
                                _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));


}

void mgnss::controllers::WheeledMotionEvent3::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();
        // Set-up hierachical controller
        WheelsControllerExtend::_createTasks(config);

        state_machine__.reset(new mgnss::higher_level::StateMachineII(_robot, config ));

        mwoibn::Vector3 pelvis;
        pelvis << 1, 1, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));

        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));

        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        _pelvis.reset(new mwoibn::robot_points::LinearPoint("pelvis", _robot));

        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl(
                             config["track"].as<std::string>(), _robot, config, *_pelvis, _robot.getLinks("base")[0]));

        _steering_ptr->subscribe(true, true, false);



        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);
        // _world_posture_ptr->addTask(*_com_ptr);

        mwoibn::VectorBool select(3);
        select << true, true, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);

        _tasks["CONTACT_POINTS"] = _steering_ptr.get();
        // _tasks["BASE_GROUND"] = _com_ptr.get();
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();

}


void mgnss::controllers::WheeledMotionEvent3::_setInitialConditions(){

        WheelsControllerExtend::_setInitialConditions();

        _pelvis_position_ptr->points().point(0).getLinearWorld(_position);
        _position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);
}


void mgnss::controllers::WheeledMotionEvent3::log(mwoibn::common::Logger& logger, double time){
  logger.add("time", time);

   logger.add("th", _robot.state.position.get()[5]);
   logger.add("r_th", _heading);
   //
       for(int i = 0; i < 3; i++){

         logger.add(std::string("cop_") + char('x'+i), _robot.centerOfPressure().get()[i]);
         logger.add(std::string("com_") + char('x'+i), _robot.centerOfMass().get()[i]);
         logger.add(std::string("r_base_") + char('x'+i), getBaseReference()[i]);
         logger.add(std::string("base_") + char('x'+i), _steering_ptr->base.get()[i]);

         for(int k = 0; k < 4; k++){

           logger.add("cp_"   + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getPointStateReference(k)[i]);

           logger.add("r_cp_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getReference()[k*3+i]);

           logger.add("full_error_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getFullError()[k*3+i]);

           logger.add("getForce_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getForce()[k*3+i]);


         }
       }


              // std::cout << "wheels position\t" << _robot.state.position.get().head<6>().transpose() << std::endl;

             // std::cout << "wheels velocity\t" << _robot.state.velocity.get().head<6>().transpose() << std::endl;


  // std::cout << _robot.state.velocity.get().head<6>().transpose() << std::endl;
  state_machine__->log(logger);

}
