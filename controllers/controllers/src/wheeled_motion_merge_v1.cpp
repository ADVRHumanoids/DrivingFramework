#include "mgnss/controllers/wheeled_motion_actions.h"
#include "mgnss/higher_level/steering_v8.h"
#include <mwoibn/hierarchical_control/tasks/cartesian_simplified_pelvis_task_v7.h>
#include <mwoibn/hierarchical_control/controllers/actions.h>

#include "mgnss/controllers/wheeled_motion_merge_v1.h"

mgnss::controllers::WheeledMotionMergeV1::WheeledMotionMergeV1(
        mwoibn::robot_class::Robot& robot, std::string config_file)
        : WheeledMotionActions(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["wheeled_motion"];
        _init(config);
}

mgnss::controllers::WheeledMotionMergeV1::WheeledMotionMergeV1(
        mwoibn::robot_class::Robot& robot, YAML::Node config)
        : WheeledMotionActions(robot)
{
        _init(config);
}

void mgnss::controllers::WheeledMotionMergeV1::_init(YAML::Node config)
{

        std::cout << "wheels allocate" << std::endl;
        _x << 1, 0, 0;
        _y << 0, 1, 0;
        _z << 0, 0, 1;
        _hierarchical_controller_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));
        _actions_ptr = dynamic_cast<mwoibn::hierarchical_control::controllers::Actions*>(_hierarchical_controller_ptr.get());
        _createTasks(config);
        _initIK(config);
        _allocate();

        _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
                                        _robot, *_steering_ptr, _support_vel, _test_steer, config["steer_open_loop"].as<double>(), config["steer_feedback"].as<double>(), config["tracking_gain"].as<double>(), _robot.rate(), config["damp_icm"].as<double>(), config["damp_sp"].as<double>(), config["steer_damp"].as<double>()));
}

void mgnss::controllers::WheeledMotionMergeV1::_createAngleTasks(YAML::Node config){

        WheeledMotionActions::_createAngleTasks(config);
}

void mgnss::controllers::WheeledMotionMergeV1::_initIK(YAML::Node config){
        std::cout << "Wheeled Motion loaded " << config["tunning"] << " tunning." << std::endl;

        _hierarchical_controller_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));
        _actions_ptr = dynamic_cast<mwoibn::hierarchical_control::controllers::Actions*>(_hierarchical_controller_ptr.get());


        if(!config["chain"])
                throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'chain'."));
                _select_ik = _robot.getDof(_robot.getLinks(config["chain"].as<std::string>()));


        config = config["tunnings"][config["tunning"].as<std::string>()];

        for(auto entry : config)
                std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

        // int task = 0;
        double ratio = config["ratio"].as<double>(); // 4
        double damp = config["damping"].as<double>();
        // Set initaial HC tasks
        _hierarchical_controller_ptr->addTask(*_constraints_ptr, config["constraints"].as<double>(), damp);
        _hierarchical_controller_ptr->addTask(_leg_steer, config["leg_steer"].as<double>() * ratio, damp);

        mwoibn::VectorN gain_base(6);
        gain_base.head<3>() = mwoibn::VectorN::Constant(3, config["base_orinetation"].as<double>() * ratio);
        gain_base[3] = config["centre_of_mass_x"].as<double>() * ratio;
        gain_base[4] = config["centre_of_mass_y"].as<double>() * ratio;
        gain_base[5] = config["base_position"].as<double>() * ratio;
        _hierarchical_controller_ptr->addTask(*_world_posture_ptr, gain_base,  damp);

        _hierarchical_controller_ptr->addTask(*_steering_ptr, config["contact_point"].as<double>() * ratio, damp);

        _leg_camber_action.reset(new mwoibn::hierarchical_control::actions::Compute(_leg_camber, config["camber"].as<double>() * ratio, config["camber_damp"].as<double>(), _actions_ptr->state().P, _actions_ptr->state().command, _actions_ptr->state().memory));

        _leg_castor_action.reset(new mwoibn::hierarchical_control::actions::Compute(_leg_castor, config["castor"].as<double>() * ratio, config["castor_damp"].as<double>(), _actions_ptr->state().P, _actions_ptr->state().command, _actions_ptr->state().memory));
        // _hierarchical_controller_ptr->addTask(_leg_camber, config["camber"].as<double>() * ratio, config["camber_damp"].as<double>());
        //_hierarchical_controller_ptr->addTask(*_leg_castor_ptr, config["castor"].as<double>() * ratio, config["castor_damp"].as<double>());

        // _actions_ptr->idleTask(_leg_castor, config["castor"].as<double>() * ratio, config["castor_damp"].as<double>());

        _leg_merge_ptr.reset(new mwoibn::hierarchical_control::actions::AnglesMerge(*_leg_camber_action, *_leg_castor_action, _actions_ptr->state(), config["camber_tolerance"].as<double>(), config["camber_speed"].as<double>(), _caster_task, *_constraints_ptr, _robot));

        _actions_ptr->addAction(*_leg_merge_ptr);

        _hierarchical_controller_ptr->update();

        // mwoibn::hierarchical_control::tasks::Merge(_leg_castor, _leg_camber);

}
