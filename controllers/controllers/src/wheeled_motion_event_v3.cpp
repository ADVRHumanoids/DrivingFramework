#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/higher_level/steering_v8.h"
#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>


void mgnss::controllers::WheeledMotionEvent3::_allocate(){

        WheelsControllerExtend::_allocate();

        _com_ref.setZero(2);

}

void mgnss::controllers::WheeledMotionEvent3::_initIK(YAML::Node config){

        WheelsController::_initIK(config);

        _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
                                _robot, *_steering_ptr, _support_vel, config["steer_open_loop"].as<double>(), config["steer_feedback"].as<double>(), config["tracking_gain"].as<double>(), _robot.rate(), config["damp_icm"].as<double>(), config["damp_sp"].as<double>(), config["steer_damp"].as<double>()));


}

void mgnss::controllers::WheeledMotionEvent3::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();
        // Set-up hierachical controller
        WheelsControllerExtend::_createTasks(config);

        mwoibn::Vector3 pelvis;
        pelvis << 0, 0, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));

        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));

        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl(
                             _robot.getLinks("wheels"), _robot, config, _robot.centerOfMass(), _robot.getLinks("base")[0]));



        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);
        _world_posture_ptr->addTask(*_com_ptr);

        mwoibn::VectorBool select(3);
        select << false, false, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);

        _tasks["CONTACT_POINTS"] = _steering_ptr.get();
        _tasks["BASE_GROUND"] = _com_ptr.get();
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

void mgnss::controllers::WheeledMotionEvent3::initLog(mwoibn::common::Logger& logger){
        mgnss::controllers::WheelsControllerExtend::initLog(logger);

        logger.addField("com_x", getComFull()[0]);
        logger.addField("com_y", getComFull()[1]);
        //
        logger.addField("r_com_x", refCom()[0]);
        logger.addField("r_com_y", refCom()[1]);

}

void mgnss::controllers::WheeledMotionEvent3::log(mwoibn::common::Logger& logger, double time){
   mgnss::controllers::WheelsControllerExtend::log(logger ,time);

   logger.addEntry("com_x", getComFull()[0]);
   logger.addEntry("com_y", getComFull()[1]);
   //
   logger.addEntry("r_com_x", refCom()[0]);
   logger.addEntry("r_com_y", refCom()[1]);



}
