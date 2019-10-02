#include "mgnss/plugins/ros_base.h"
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>


void mgnss::plugins::RosBase::connect(std::string name)
{
        _plugin_ptr->name = name;
        ros::init(std::map<std::string, std::string>(), _plugin_ptr->name);
        _plugin_ptr->n.reset(new ros::NodeHandle());
}

void mgnss::plugins::RosBase::connect(int argc, char** argv, std::string name)
{
        _plugin_ptr->name = name;
        ros::init(argc, argv, _plugin_ptr->name);
        _plugin_ptr->n.reset(new ros::NodeHandle());
}

std::string mgnss::plugins::RosBase::_configFile(){
        std::string config_file;
        // Read the config file from the ros parameter server
        if (!_plugin_ptr->n->getParam("/mwoibn_config", config_file) && !_plugin_ptr->n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        return config_file;
}


bool mgnss::plugins::RosBase::init()
{
        // Read the configuration files
        YAML::Node config, plugin_config;

        std::string config_file = _configFile();
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        // get the robot id
        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);
        // create the robot instance
        _plugin_ptr->shareRobots()[robot_id] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>()));

        // create the logger
        _plugin_ptr->logger_ptr.reset(new mwoibn::common::RosLogger(_plugin_ptr->name));

        // create the module 
        _plugin_ptr->initModule(config, plugin_config);
        // initialize the logger - allocate the memory, declare the logged fields
        _plugin_ptr->controller_ptr->startLog(*(_plugin_ptr->logger_ptr));

        return true;
}


bool mgnss::plugins::RosBase::init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name){

  _plugin_ptr->n = n;
  _plugin_ptr->name = name;

  // Read the configuration files
  YAML::Node config, plugin_config;

  std::string config_file = _configFile();

  std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);
  // get the robot id
  std::string robot_name = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);

  // check if the robot already exists
  if (share_robots.count(robot_name)){
    _plugin_ptr->shareRobots()[robot_name] = share_robots[robot_name]; // if yes, share the pointer to the robot with the module
    // if yes, add the controllers to the robot
    _plugin_ptr->shareRobots()[robot_name]->loadControllers(config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>());
  }
  else{
    // if no, create the new robot
    _plugin_ptr->shareRobots()[robot_name] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>()));
    // return the pointer to the robot to the shared space
    share_robots[robot_name] = _plugin_ptr->shareRobots()[robot_name];
  }
  // share the pointer to the logger
  _plugin_ptr->logger_ptr = logger_ptr;
  // init the module
  _plugin_ptr->initModule(config, plugin_config, share);
  /** Set logger prefix to the name of the loaded configuration.
   *  This is done to avoid the variables to be overwritten if two plugins record the data with the same identifier (e.g. time)
   */ 
  logger_ptr->prefix(name);
  // preallocate the memmory in the logger for this plugin
  _plugin_ptr->controller_ptr->log(*(_plugin_ptr->logger_ptr),0);

}
