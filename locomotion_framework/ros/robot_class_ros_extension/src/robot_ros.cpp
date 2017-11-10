#include "mwoibn/robot_class/robot_ros.h"
#include "mwoibn/point_handling/raw_positions_handler.h"

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>
mwoibn::robot_class::RobotRos::RobotRos(std::string config_file,
                                        std::string config_name,
                                        std::string secondary_file)
    : mwoibn::robot_class::RobotRos::Robot()
{

  // Creat final config file from obtained parameter files
  YAML::Node config = getConfig(config_file, secondary_file);

  try
  {
    _init(config, config_name);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("\nconfig_file:\t" + config_file +
                                std::string("\nrobot:\t") + config_name +
                                std::string("\n") + e.what()));
  }
}

YAML::Node mwoibn::robot_class::RobotRos::_init(YAML::Node config,
                                                std::string config_name)
{

  YAML::Node config_robot = _readRobotConfig(config, config_name);
  // read ROS specific configuration
  config = config["ros"];

  mwoibn::robot_class::Robot::_init(_readUrdf(config["source"]),
                                    _readSrdf(config["source"]));

  config_robot["actuators"]["source"] = config["source"]["actuators"];
  config_robot["contacts"]["source"] = config["source"]["contacts"];

  config_robot["mapping"] = config["mapping"];

  _readActuators(config_robot);
  _readContacts(config_robot);

  _loadConfig(config["feedback"], config_robot["feedback"]);
  _loadConfig(config["controller"], config_robot["controller"]);

  if (!config_robot["rate"])
    throw(std::invalid_argument("Desired frequency not defined."));

  _rate_ptr.reset(new ros::Rate(config_robot["rate"].as<double>()));

  return config_robot;
}

std::string mwoibn::robot_class::RobotRos::_readUrdf(YAML::Node config)
{
  if (!config["urdf"])
    throw std::invalid_argument(
        std::string("Pleas define an urdf source in the yaml file."));

  std::string urdf;
  std::cout << "URDF read from param:\t" << config["urdf"].as<std::string>()
            << std::endl;

  if (!_node.getParam(config["urdf"].as<std::string>(), urdf))
    throw std::invalid_argument(std::string("Could not retrieve a parameter ") +
                                config["urdf"].as<std::string>() +
                                std::string(" from parameter server."));
  return urdf;
}

std::string mwoibn::robot_class::RobotRos::_readSrdf(YAML::Node config)
{

  std::string srdf = "";
  if (config["srdf"].as<std::string>() != "" &&
      !_node.getParam(config["srdf"].as<std::string>(), srdf))

    throw std::invalid_argument(std::string("Could not retrieve a parameter ") +
                                config["source"]["srdf"].as<std::string>() +
                                std::string(" from parameter server."));

  return srdf;
}

void mwoibn::robot_class::RobotRos::_readContacts(YAML::Node config)
{

  std::string file =
      _readConfigString(config["contacts"], config["name"].as<std::string>());
  if (file.empty())
    return;

  YAML::Node contacts =
      YAML::Load(file)[config["name"].as<std::string>()]["contacts"];
  contacts["settings"] = config["contacts"];
  _loadContacts(contacts);
}

void mwoibn::robot_class::RobotRos::_readActuators(YAML::Node config)
{
  std::string file =
      _readConfigString(config["actuators"], config["name"].as<std::string>());
  if (file.empty())
    return;

  YAML::Node actuators =
      YAML::Load(file)[config["name"].as<std::string>()]["actuators"];
  actuators["settings"] = config["actuators"];
  _loadActuators(actuators);
}

std::string mwoibn::robot_class::RobotRos::_readConfigString(YAML::Node config,
                                                             std::string name)
{

  // check if config is correct
  if (!config["read"])
    throw std::invalid_argument(std::string("Please specify config status"));

  // whether continue or not
  if (!config["read"].as<bool>())
    return "";

  // if knows which actuators should be loaded
  if (name.empty())
    throw std::invalid_argument(std::string("Please, define robot name"));

  // read contact configuration from ros parameter server
  std::string prefix =
      (config["source"]) ? config["source"].as<std::string>() + "/" : "";

  std::string string_config;

  if (!_node.getParam(prefix, string_config))
    throw std::invalid_argument(std::string("Wrong ") + name + std::string(" source: ") +
                                prefix);

  return string_config;
}

bool mwoibn::robot_class::RobotRos::_initUrdf(YAML::Node config,
                                              std::string& source)
{

  std::stringstream errMsg;
  urdf::Model urdf;

  source = _readUrdf(config);

  if (!urdf.initString(source))
  {
    errMsg << "Could not load urdf description  for mapping " +
                  config["name"].as<std::string>();
    throw(std::invalid_argument(errMsg.str().c_str()));
  }
  return (urdf.getRoot()->child_joints[0]->type == urdf::Joint::FLOATING)
             ? false
             : true;
  ;
}

RigidBodyDynamics::Model
mwoibn::robot_class::RobotRos::_initModel(bool is_static,
                                          const std::string& source)
{
  RigidBodyDynamics::Model model;
  if (!RigidBodyDynamics::Addons::URDFReadFromString(source.c_str(), &model,
                                                     !is_static, false))
    throw std::invalid_argument(
        std::string("Error loading model from string for mapping "));

  return model;
}
