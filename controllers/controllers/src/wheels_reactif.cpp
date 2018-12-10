#include "mgnss/controllers/wheels_reactif.h"
#include "mgnss/higher_level/steering_reactif.h"
#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>


void mgnss::controllers::WheelsReactif::_allocate(){

        WheelsControllerExtend::_allocate();
}

void mgnss::controllers::WheelsReactif::_initIK(YAML::Node config){

        WheelsController::_initIK(config);

        _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
                _robot, *_steering_ptr, _support_vel, config["steer_open_loop"].as<double>(), config["steer_feedback"].as<double>(), config["tracking_gain"].as<double>(), _robot.rate(), config["damp_icm"].as<double>(), config["damp_sp"].as<double>(), config["steer_damp"].as<double>()));

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


void mgnss::controllers::WheelsReactif::initLog(mwoibn::common::Logger& logger){
  mgnss::controllers::WheelsControllerExtend::initLog(logger);

  logger.addField("com_x", _robot.centerOfMass().get()[0]);
  logger.addField("com_y", _robot.centerOfMass().get()[1]);

  logger.addField("base_x", getBaseGroundX());
  logger.addField("base_y", getBaseGroundY());

  logger.addField("r_base_x", _pelvis_position_ptr->getReference()[0]);
  logger.addField("r_base_y", _pelvis_position_ptr->getReference()[1]);

}


  void mgnss::controllers::WheelsReactif::log(mwoibn::common::Logger& logger, double time){
     mgnss::controllers::WheelsControllerExtend::log(logger ,time);

     logger.addEntry("com_x", _robot.centerOfMass().get()[0]);
     logger.addEntry("com_y", _robot.centerOfMass().get()[1]);

     logger.addEntry("base_x", getBaseGroundX());
     logger.addEntry("base_y", getBaseGroundY());

     logger.addEntry("r_base_x", _pelvis_position_ptr->getReference()[0]);
     logger.addEntry("r_base_y", _pelvis_position_ptr->getReference()[1]);

}
