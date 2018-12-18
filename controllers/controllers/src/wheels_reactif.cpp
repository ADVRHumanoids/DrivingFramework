#include "mgnss/controllers/wheels_reactif.h"
#include "mgnss/higher_level/steering_reactif.h"
#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>


void mgnss::controllers::WheelsReactif::_allocate(){

        WheelsControllerExtend::_allocate();
}

void mgnss::controllers::WheelsReactif::_initIK(YAML::Node config){

        WheelsController::_initIK(config);

        YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

        _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
                  _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));
}

void mgnss::controllers::WheelsReactif::_createTasks(YAML::Node config){

        // Set-up hierachical controller
        _name = config["name"].as<std::string>();

        WheelsControllerExtend::_createTasks(config);

        mwoibn::Vector3 pelvis;
        pelvis << 1, 1, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));
        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl(
                        _robot.getLinks("wheels"), _robot, config, _robot.centerOfMass(), _robot.getLinks("base")[0]));

        _steering_ptr->subscribe(true, true, false);
        _tasks["CONTACT_POINTS"] = _steering_ptr.get();

        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);

        //_tasks["BASE_GROUND"] = *_pelvis_position_ptr;
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();

        mwoibn::VectorBool select(3);
        select << true, true, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);

}

void mgnss::controllers::WheelsReactif::_setInitialConditions(){

        WheelsControllerExtend::_setInitialConditions();

        _pelvis_position_ptr->points().point(0).getLinearWorld(_position);
        _position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
}


void mgnss::controllers::WheelsReactif::compute()
{
        mgnss::controllers::WheelsController::compute();
        _correct();
}


  void mgnss::controllers::WheelsReactif::log(mwoibn::common::Logger& logger, double time){
     mgnss::controllers::WheelsControllerExtend::log(logger ,time);

     logger.add("com_x", _robot.centerOfMass().get()[0]);
     logger.add("com_y", _robot.centerOfMass().get()[1]);

     logger.add("base_x", getBaseGroundX());
     logger.add("base_y", getBaseGroundY());

     logger.add("r_base_x", _pelvis_position_ptr->getReference()[0]);
     logger.add("r_base_y", _pelvis_position_ptr->getReference()[1]);

}
