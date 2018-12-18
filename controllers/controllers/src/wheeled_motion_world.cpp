#include "mgnss/controllers/wheeled_motion_world.h"
#include "mgnss/higher_level/steering_v5.h"
#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>

void mgnss::controllers::WheeledMotionWorld::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();
        WheelsControllerExtend::_createTasks(config);

        // Set-up hierachical controller
        //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);

        mwoibn::Vector3 pelvis;
        pelvis << 0, 0, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));

        _pelvis_position_ptr.reset(
                 new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph, pelvis));

        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));

        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);
        _world_posture_ptr->addTask(*_com_ptr);

        mwoibn::VectorBool select(3);
        select << false, false, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);


        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl(
                        _robot.getLinks("wheels"), _robot, config, _world, _robot.getLinks("base")[0]));
        _steering_ptr->subscribe(true, true, false);

        _tasks["CONTACT_POINTS"] = _steering_ptr.get();
        _tasks["BASE_GROUND"] = _com_ptr.get();
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();


}

void mgnss::controllers::WheeledMotionWorld::_initIK(YAML::Node config){
        WheelsController::_initIK(config);

        YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

        _steering_ref_ptr.reset(new mgnss::higher_level::Steering5(
                                _robot, *_steering_ptr, steering["icm"].as<double>(), steering["sp"].as<double>(), _robot.rate(), steering["damp"].as<double>()));

}

void mgnss::controllers::WheeledMotionWorld::_allocate(){

        WheelsControllerExtend::_allocate();

        _com_ref.setZero(2);

}

void mgnss::controllers::WheeledMotionWorld::_setInitialConditions(){

        WheelsControllerExtend::_setInitialConditions();

        _position = _pelvis_position_ptr->points().getPointStateWorld(0);
        _position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);
}

void mgnss::controllers::WheeledMotionWorld::log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);
        mgnss::controllers::WheelsControllerExtend::log(logger ,time);

}
