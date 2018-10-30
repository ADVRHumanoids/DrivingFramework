#ifndef __MGNSS_ROS_PLUGINS_CENTRALIZED_H
#define __MGNSS_ROS_PLUGINS_CENTRALIZED_H

#include "mgnss/plugins/ros_base.h"
#include "mgnss/controllers/online_centralized_controller.h"
#include "mgnss/ros_callbacks/centralized.h"
#include <mwoibn/loaders/robot.h>

namespace mgnss {
namespace nrt_software {
namespace plugins {

class Centralized : public mgnss::plugins::RosBase
{

public:
Centralized(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv){
        _init(argc, argv);
}

virtual std::string _checkConfig(YAML::Node plugin_config, std::string config_file){
        std::string secondary_file =  RosBase::_checkConfig(plugin_config, config_file);

        if (!plugin_config["reference"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find reference robot configuration in module parameters."));

        return secondary_file;
}

bool init()
{
        YAML::Node config, plugin_config;

        std::string config_file = _loadConfig(config, plugin_config);

        std::string secondary_file = _checkConfig(plugin_config, config_file);

        _loadRobot(config_file, secondary_file, config, plugin_config);
        _reference_ptr.reset(mwoibn::loaders::Robot::create(config,  plugin_config["reference"].as<std::string>()).release());

        _initModule(config, plugin_config);

        return true;
}

void start(double time)
{
        robot().update();
        robot().wait();
        robot().command.position.set(robot().state.position.get()); //initialize with current position
        mgnss::plugins::RosBase::start(time);
}

mgnss::controllers::OnlineCentralizedController& get(){
        return static_cast<mgnss::controllers::OnlineCentralizedController&>(*_controller_ptr);
}
mwoibn::robot_class::Robot& robot(){
        return *_robot_ptr;
} // ?? do I need it

virtual ~Centralized(){
}


protected:
//ros::ServiceServer _srv_reset, _srv_switch;
//ros::Subscriber _sub_rt;
std::unique_ptr<mwoibn::robot_class::Robot> _reference_ptr;

virtual std::string _setName(){
        return "centralized";
}

virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::OnlineCentralizedController(*_robot_ptr, *_reference_ptr, config));
}

virtual void _initCallbacks(){
/*        _sub_rt = n.subscribe<custom_messages::CustomCmnd>(
                "CoM_regulator/contacts", 1,
                boost::bind(&getContacts, _1, &_robot, &_robot));
 */
        //  // debbuging purposes, switch between motor side and link side reference
/*        _srv_switch = n.advertiseService<std_srvs::SetBool::Request,
                                         std_srvs::SetBool::Response>(
                "centralized_controller/motor_side_reference",
                boost::bind(&setMotorSideReference, _1, _2, &_robot, &_robot,
                            &motor_side));
 */
        // debbuging purposes, set current robot position as new reference
/*        _srv_reset = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                "centralized_controller/reset_reference",
                boost::bind(&resetReference, _1, _2, &_robot, &_robot,
                            &motor_side));
 */
}

};

}
}
}
#endif // RT_MY_TEST_H
