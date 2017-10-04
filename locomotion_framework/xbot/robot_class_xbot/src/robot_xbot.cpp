#include "mwoibn/robot_class/robot_xbot.h"
//#include "mwoibn/point_handling/raw_positions_handler.h"

mwoibn::robot_class::RobotXBot::RobotXBot(std::string config_file, std::string config_name, std::string secondary_file){

  // Creat final config file from obtained parameter files
  YAML::Node config = _getConfig(config_file, secondary_file);

  _init(config, config_name); // thanks to this function more config files can
                              // be easily supported
}



YAML::Node mwoibn::robot_class::RobotXBot::_init(YAML::Node config,
                                          std::string config_name)
{
  // retrive specific robot configuration to be loaded
  YAML::Node config_robot = _readRobotConfig(config, config_name);

  // read ROS specific configuration
  config = config["xbot"];

  mwoibn::robot_class::Robot::_init(_readUrdf(config["source"]),
                                    _readSrdf(config["source"]), true);

  config_robot["actuators"]["source"] = config["source"]["actuators"];
  config_robot["contacts"]["source"] = config["source"]["contacts"];

  _readActuators(config_robot);
  _readContacts(config_robot);

  _loadConfig(config["feedback"], config_robot["feedback"]);
  _loadConfig(config["controller"], config_robot["controller"]);

  return config_robot;
}


std::string mwoibn::robot_class::RobotXBot::_readUrdf(YAML::Node config)
{
  if (!config["urdf"])
    throw std::invalid_argument(
        std::string("Pleas define an urdf source in the yaml file."));

  return config["urdf"].as<std::string>();
}

std::string mwoibn::robot_class::RobotXBot::_readSrdf(YAML::Node config)
{

  if (config["srdf"] && config["srdf"].as<std::string>() != "")
    return config["srdf"].as<std::string>();

  return "";
}

void mwoibn::robot_class::RobotXBot::_readActuators(YAML::Node config)
{
  if(!config["actuators"]["source"])
   return;

  YAML::Node actuators = YAML::LoadFile(config["actuators"]["source"].as<std::string>())[config["name"].as<std::string>()]["actuators"];

  actuators["settings"] = config["actuators"];
  _loadActuators(actuators);
}

void mwoibn::robot_class::RobotXBot::_readContacts(YAML::Node config)
{
  if(!config["contacts"]["source"])
   return;

  std::cout << config["contacts"]["source"].as<std::string>() << std::endl;
  YAML::Node contacts = YAML::LoadFile(config["contacts"]["source"].as<std::string>())[config["name"].as<std::string>()]["contacts"];

  contacts["settings"] = config["contacts"];
  _loadContacts(contacts);

}
