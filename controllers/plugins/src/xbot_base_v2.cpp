#include "mgnss/plugins/xbot_base_v2.h"
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
