#include "mgnss/plugins/xbot_base_v2.h"
#include <mwoibn/communication_modules/shared.h>

std::string mgnss::plugins::XbotBaseUnify::_configFile(XBot::Handle::Ptr handle){
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());


  // Read path to the config file from the XBotCore config file
  if (!config["config_file"])
          throw std::invalid_argument(handle->getPathToConfigFile() +
                                      std::string("\t Please specify MWOIBN config file."));

  return config["config_file"].as<std::string>();
}



bool mgnss::plugins::XbotBaseUnify::init_control_plugin(XBot::Handle::Ptr handle)
{
        std::cout << "name\t" << _plugin_ptr->name << std::endl;

        connect(handle); // connect with ROS
        YAML::Node config, plugin_config;
        // Read the configuration files
        std::string config_file = _configFile(handle);
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        // get the robot id
        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);
        // create the robot instance        
        _plugin_ptr->shareRobots()[robot_id] = std::make_shared<mwoibn::robot_class::RobotXBotRT>(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>(), handle->getSharedMemory());

        // create the logger        
        _plugin_ptr->logger_ptr.reset(new mwoibn::common::XbotLogger(_plugin_ptr->name));

        // create the module 
        _plugin_ptr->initModule(config, plugin_config);
        /** Set logger prefix to the name of the loaded configuration.
         *  This is done to avoid the variables to be overwritten if two plugins record the data with the same identifier (e.g. time)
         */ 
        _plugin_ptr->logger_ptr->prefix(_plugin_ptr->name);
        // allocate memmory for the logger
        _plugin_ptr->controller_ptr->startLog(*(_plugin_ptr->logger_ptr.get()));

        return true;
}

bool mgnss::plugins::XbotBaseUnify::init_control_plugin(XBot::Handle::Ptr handle, robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<XBot::RosUtils::RosHandle> n, mwoibn::communication_modules::Shared& share, std::string name)
{
        _plugin_ptr->name = name;
        std::cout << "load plugin configuration\t" << _plugin_ptr->name << std::endl;

        connect(handle); // connect with ROS
        YAML::Node config, plugin_config;
        // Read the configuration files
        std::string config_file = _configFile(handle);
        std::string secondary_file = _plugin_ptr->readConfig(config_file, config, plugin_config);

        // get the robot id
        std::string robot_id = _plugin_ptr->readRobot(config_file, secondary_file, config, plugin_config);

        // check if the robot already exists
        if (share_robots.count(robot_id)){
          std::cout << "share robot" << std::endl;
          // if yes, add the controllers to the robot
          share_robots[robot_id]->loadControllers(config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>(), handle->getSharedMemory());

        }
        else{
          std::cout << "create robot" << std::endl;
          // if no, generate the new robot
          share_robots[robot_id] = std::make_shared<mwoibn::robot_class::RobotXBotRT>(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>(), handle->getSharedMemory());
        }
        // share the pointer to the robot
        _plugin_ptr->shareRobots()[robot_id] = share_robots[robot_id];

        // share the pointer to the logger
        _plugin_ptr->logger_ptr = logger_ptr;
        // init the module
        _plugin_ptr->initModule(config, plugin_config, share);
        /** Set logger prefix to the name of the loaded configuration.
         *  This is done to avoid the variables to be overwritten if two plugins record the data with the same identifier (e.g. time)
         */ 
        _plugin_ptr->logger_ptr->prefix(_plugin_ptr->name);
        // allocate memmory for the logger
        _plugin_ptr->controller_ptr->log(*(_plugin_ptr->logger_ptr),0);
        return true;
}
