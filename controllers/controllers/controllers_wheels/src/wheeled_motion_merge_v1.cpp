#include "mgnss/controllers/wheeled_motion_merge_v1.h"
#include <mwoibn/hierarchical_control/actions/compute.h>

mgnss::controllers::WheeledMotionMergeV1::WheeledMotionMergeV1(
        mwoibn::robot_class::Robot& robot, std::string config_file, std::string name)
        : WheeledMotionEvent3(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;
        _create(config);
}

mgnss::controllers::WheeledMotionMergeV1::WheeledMotionMergeV1(
        mwoibn::robot_class::Robot& robot, YAML::Node config)
        : WheeledMotionEvent3(robot)
{
        _create(config);
}


void mgnss::controllers::WheeledMotionMergeV1::_createTasks(YAML::Node config){

        WheeledMotionEvent3::_createTasks(config);

        std::shared_ptr<mwoibn::hierarchical_control::actions::Compute> camber_ = _taskAction("CAMBER", config["tunnings"][config["tunning"].as<std::string>()]);

        std::shared_ptr<mwoibn::hierarchical_control::actions::Compute> caster_ = _taskAction("CASTER", config["tunnings"][config["tunning"].as<std::string>()]);

        _actions["CAMBER_MERGE"] = camber_;
        _actions["CASTER_MERGE"] = caster_;

        _merge_ptr = std::make_shared<mwoibn::hierarchical_control::actions::AnglesMerge>(*camber_, *caster_, _ik_ptr->state, config["camber_tolerance"].as<double>(), config["camber_speed"].as<double>(), _leg_tasks["CAMBER"].second, *_constraints_ptr, _robot);

        _actions["CAMBER"] = _merge_ptr;

}
