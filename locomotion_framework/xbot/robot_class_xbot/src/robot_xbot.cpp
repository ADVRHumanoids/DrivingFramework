#include "mwoibn/robot_class/robot_xbot.h"
//#include "mwoibn/point_handling/raw_positions_handler.h"

mwoibn::robot_class::RobotXBot::RobotXBot(std::string config_file,
                                          std::string config_name,
                                          std::string secondary_file)
{

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
  config_robot["controller"]["source"] = config["source"]["controller"];

  config_robot["mapping"] = config["mapping"];

  _readActuators(config_robot);
  _readContacts(config_robot);
  _readController(config_robot);

  _loadConfig(config["feedback"], config_robot["feedback"]);
  _loadConfig(config["controller"], config_robot["controller"]);

  return config_robot;
}

std::string mwoibn::robot_class::RobotXBot::_readUrdf(YAML::Node config)
{
  if (!config["urdf"])
    throw std::invalid_argument(
        std::string("Please define an urdf source in the yaml file."));

  if (!config["urdf"]["file"])
    throw(std::invalid_argument(
        "Please define an urdf source in the yaml file.\n"));

  std::string file = "";
  if (config["urdf"]["path"])
    file = config["urdf"]["path"].as<std::string>();

  file += config["urdf"]["file"].as<std::string>();

  return file;
}

std::string mwoibn::robot_class::RobotXBot::_readSrdf(YAML::Node config)
{

  if (!config["srdf"])
    return "";
  if (config["srdf"]["file"].as<std::string>() == "")
    return "";

  std::string file = "";
  if (config["srdf"]["path"])
    file = config["srdf"]["path"].as<std::string>();

  file += config["srdf"]["file"].as<std::string>();

  return file;
}

void mwoibn::robot_class::RobotXBot::_readActuators(YAML::Node config)
{

  std::string file = _getFile(config["actuators"], "actuators");
  if (file == "") return;

  YAML::Node actuators =
      YAML::LoadFile(file)[config["name"].as<std::string>()]["actuators"];

  actuators["settings"] = config["actuators"];
  _loadActuators(actuators);
}

void mwoibn::robot_class::RobotXBot::_readContacts(YAML::Node config)
{
  std::string file = _getFile(config["contacts"], "contacts");
  if (file == "") return;

  YAML::Node contacts =
      YAML::LoadFile(file)[config["name"].as<std::string>()]["contacts"];

  contacts["settings"] = config["contacts"];
  _loadContacts(contacts);
}

void mwoibn::robot_class::RobotXBot::_readController(YAML::Node config)
{

  std::string file = _getFile(config["controller"], "controller");
  if (file == "") return;

  config["controller"]["gains"] =
        YAML::LoadFile(file)[config["name"].as<std::string>()];
  // config["controller"].remove("source");
}

std::string mwoibn::robot_class::RobotXBot::_getFile(YAML::Node config,
                                                     std::string name)
{
  if (!config["source"] || !config["source"]["file"] ||
      config["source"]["file"].as<std::string>() == "")
  {
    std::cout << "WARNING: no " << name
              << " gains has been defined. Continue with defualt" << std::endl;
    return "";
  }

  if (!config["source"]["file"])
    throw(std::invalid_argument(std::string("Please define a source of ") +
                                name + std::string(" data.\n")));

  std::string file = "";
  if (config["source"]["path"])
    file = config["source"]["path"].as<std::string>();

  file += config["source"]["file"].as<std::string>();

  std::cout << name << " source:\t" << file << std::endl;
  return file;
}
