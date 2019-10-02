#ifndef __MGNSS_PLUGINS_ROS_SHARED_H
#define __MGNSS_PLUGINS_ROS_SHARED_H

#include "mgnss/modules/base.h"
#include "mgnss/plugins/ros_factory.h"
#include <mwoibn/common/ros_logger.h>
#include <mwoibn/communication_modules/shared.h>
#include <ros/ros.h>

namespace mgnss
{
namespace plugins
{
//! ROS plugin that allows to run the multiple RosBase plugins in the same shared space
/*  It coordinates updates on the robot kineamtics between the plugins to avoid multiple updates on the same kinematics in the loop. 
 *  It allows to share current/desired state, and robot points within the shared space without relying on the middleware.
 */
class RosShared
{

public:

RosShared() {}

virtual ~RosShared(){}

//! establish a connection with the ROS
virtual void connect(int argc, char** argv, std::string name);
//! establish a connection with the ROS
virtual void connect(std::string name);

//! creates all modules, load robot models and initializes the communication
virtual bool init();

//! executed when the plugin shut downs. It shut downs all the modules and handles the logger shut down.
virtual bool close();

//! executed when the plugin is turned on. 
/** Checks the validity of all feedbacks and sets initial conditions */
virtual void start(double time);

//! executed when the plugin has been stoped. Stops all the modules.
virtual void stop();
//! executed at each control step. Updates/Run modules and robot models
virtual void control_loop(double time);

protected:
//! update the modules time step
virtual void _setRate(double period){
    for(auto& controller: _controller_ptrs)
        controller->setRate(period);
}


std::vector<std::unique_ptr<mgnss::modules::Base>> _controller_ptrs; // vector of all modules loaded in the shared space
std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _robot_ptr; // map of robots in the shared space to thier ids
std::shared_ptr<mwoibn::common::Logger> _logger_ptr; // pointer to the logger
mwoibn::communication_modules::Shared _shared; // shared space

//! Read the config from the ROS parameter server
virtual std::string _loadConfig(YAML::Node& config, YAML::Node& plugin_config);
//! creates all the modules
virtual void _initModules(YAML::Node plugin_config);
//! check if all the necessary parameters have been defined in the configuration file
virtual void _checkConfig(YAML::Node plugin_config, std::string config_file);



std::shared_ptr<ros::NodeHandle> _n; // pointer to the ROS handle

bool _initialized = false, _valid = false, _rate = false; // initialize all the checkup falgs
std::string _name = "";
double _start; // keeps the time the plugin was started

//! function that runs at the beginning of the programm to set up the initial conditions
virtual void _init(double time);
//! helper function that coordinates the updates on the robot kinematics
virtual void _resetUpdates();

};
}
}
#endif // RT_MY_TEST_H
