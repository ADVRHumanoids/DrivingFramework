#include "mgnss/plugins/xbot_base_v2.h"
#include <mwoibn/communication_modules/shared.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)



std::string mgnss::plugins::XbotBaseUnify::_configFile(XBot::Handle::Ptr handle){
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());


  // Read path to mwoibn config file
  if (!config["config_file"])
          throw std::invalid_argument(handle->getPathToConfigFile() +
                                      std::string("\t Please specify MWOIBN config file."));

  return config["config_file"].as<std::string>();
}



bool mgnss::plugins::XbotBaseUnify::init_control_plugin(XBot::Handle::Ptr handle)
{
        std::cout << "name\t" << _plugin_ptr->name << std::endl;

        connect(handle);
        YAML::Node config, plugin_config;
        // This is load config
        std::string config_file = _configFile(handle);
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);
        _plugin_ptr->shareRobots()[robot_id] = std::make_shared<mwoibn::robot_class::RobotXBotRT>(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>(), handle->getSharedMemory());

        _plugin_ptr->logger_ptr.reset(new mwoibn::common::XbotLogger(_plugin_ptr->name));

        _plugin_ptr->initModule(config, plugin_config);
        _plugin_ptr->controller_ptr->startLog(*(_plugin_ptr->logger_ptr.get()));

        return true;
}

bool mgnss::plugins::XbotBaseUnify::init_control_plugin(XBot::Handle::Ptr handle, robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<XBot::RosUtils::RosHandle> n, mwoibn::communication_modules::Shared& share, std::string name)
{
        _plugin_ptr->name = name;
        std::cout << "load plugin configuration\t" << _plugin_ptr->name << std::endl;

        connect(handle);
        YAML::Node config, plugin_config;
        // This is load config
        std::string config_file = _configFile(handle);
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);

        if (share_robots.count(robot_id)){
            _plugin_ptr->shareRobots()[robot_id] = share_robots[robot_id];
            _plugin_ptr->shareRobots()[robot_id]->loadControllers(config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>());
        }
        else{
          _plugin_ptr->shareRobots()[robot_id] = std::make_shared<mwoibn::robot_class::RobotXBotRT>(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>(), handle->getSharedMemory());
          share_robots[robot_id] = _plugin_ptr->shareRobots()[robot_id];
        }

        _plugin_ptr->logger_ptr = logger_ptr;
        _plugin_ptr->initModule(config, plugin_config, share);
        _plugin_ptr->controller_ptr->log(*(_plugin_ptr->logger_ptr),0);
        return true;
}
