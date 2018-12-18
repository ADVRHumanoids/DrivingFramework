#include "mgnss/plugins/ros_base.h"
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)


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
        // init();
        // ros::NodeHandle n;
}

std::string mgnss::plugins::RosBase::_configFile(){
        std::string config_file;
        // Read MWOIBN config file
        if (!_plugin_ptr->n->getParam("/mwoibn_config", config_file) && !_plugin_ptr->n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        return config_file;
}


bool mgnss::plugins::RosBase::init()
{
        YAML::Node config, plugin_config;

        std::string config_file = _configFile();
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);
        _plugin_ptr->shareRobots()[robot_id] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>()));

        _plugin_ptr->logger_ptr.reset(new mwoibn::common::RosLogger(_plugin_ptr->name));

        _plugin_ptr->initModule(config, plugin_config);
        _plugin_ptr->controller_ptr->startLog(*(_plugin_ptr->logger_ptr));

        return true;
}


bool mgnss::plugins::RosBase::init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name){

  _plugin_ptr->n = n;
  _plugin_ptr->name = name;

  YAML::Node config, plugin_config;

  std::string config_file = _configFile();
  //_loadConfig(config_file, config, plugin_config);

  std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);
  std::string robot_name = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);

  if (share_robots.count(robot_name)){
    _plugin_ptr->shareRobots()[robot_name] = share_robots[robot_name];
    _plugin_ptr->shareRobots()[robot_name]->loadControllers(config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>());
  }
  else{
    //std::string robot_id = _readRobot(config_file, secondary_file, config, plugin_config);
    _plugin_ptr->shareRobots()[robot_name] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>()));
    share_robots[robot_name] = _plugin_ptr->shareRobots()[robot_name];
  }
  _plugin_ptr->logger_ptr = logger_ptr;
  _plugin_ptr->initModule(config, plugin_config, share);
  _plugin_ptr->controller_ptr->log(*(_plugin_ptr->logger_ptr),0);

}
