#include "mwoibn/robot_class/robot_xbot_rt.h"

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(
    std::string config_file, std::string robot_reference,
    XBot::SharedMemory::Ptr shared_memory)
    : RobotXBotFeedback()
{

  YAML::Node config = YAML::LoadFile(config_file);

  if (!config["mwoibnRobot"])
    throw(std::invalid_argument(
        "Please define robot configuration parameters in [mwoibnRobot].\n"));

  config = config["mwoibnRobot"];

  if (!config[robot_reference])
    throw(std::invalid_argument(
        "Please define which robot configuration should be considered.\n"));

  config = config[robot_reference];

  if (!config["config_file"])
    throw(std::invalid_argument(
        "Please define robot configuration files in [mwoibnRobot][" +
        robot_reference + "][config_file].\n"));
  if (!config["config_name"])
    throw(
        std::invalid_argument("Please define which robot configuration should "
                              "be loaded in [mwoibnRobot][config_name].\n"));

  config_file = config["config_file"].as<std::string>();
  std::string config_name = config["config_name"].as<std::string>();
  std::string secondary_file = "";
  if (config["secondary_file"])
  {
    secondary_file = config["secondary_file"].as<std::string>();
  }
  config = getConfig(config_file,
                      secondary_file); // this is done twice with this robot

  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name);
    //    _init(config, robot);

    _init(config, robot, shared_memory);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config file:\t" + config_file +
                                std::string("\n robot configuration: \t ") +
                                config_name + std::string("\nerror:\t") +
                                e.what()));
  }

  _sense = true;
  }

void mwoibn::robot_class::RobotXBotRT::_init(
    YAML::Node config, YAML::Node robot, XBot::SharedMemory::Ptr shared_memory)
{
  if (!config["source"])
    throw(std::invalid_argument("Please define sources for XBot.\n"));

  if (!config["source"]["config"]["file"])
    throw(std::invalid_argument(
        "Please define path to XBot configuration file for feedback.\n"));

  std::string file = "";
  if (config["source"]["config"]["path"]) file = config["source"]["config"]["path"].as<std::string>();

  file += config["source"]["config"]["file"].as<std::string>();

  _robot = XBot::RobotInterface::getRobot(file);

  biMaps().addMap(makeBiMap(getLinks(_robot->getEnabledJointNames()), "XBOT"));
  //  _xbot_map = biMaps().getId("XBOT");

  _loadFeedbacks(robot["feedback"], shared_memory);
  _loadControllers(robot["controller"], shared_memory);

  update();
}

void mwoibn::robot_class::RobotXBotRT::_loadFeedbacks(
    YAML::Node config, XBot::SharedMemory::Ptr shared_memory)
{

  RobotXBotFeedback::_loadFeedbacks(config);

  for (auto entry : config)
  {

    if (!_loadFeedback(entry.second, entry.first.as<std::string>()))
      continue;
    BiMap map = readBiMap(entry.second["dofs"]);

    if (!entry.second["layer"])
      throw(std::invalid_argument("Please defined type of a feedback " +
                                  entry.first.as<std::string>()));

    if (entry.second["layer"].as<std::string>() == "RT")
    {
      if (entry.second["space"].as<std::string>() == "JOINT")
      {
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotFeedbackShared(
                    state, map, entry.second, shared_memory)));
        continue;
      }
    }
    if (entry.second["layer"].as<std::string>() == "NRT")
    {
      if (entry.second["space"].as<std::string>() == "JOINT")
      {
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotFeedbackFromNRT(
                    state, map, entry.second)));

        continue;
      }
      if (entry.second["space"].as<std::string>() == "OPERATIONAL")
      {
        _getDefaultPosition(entry.second, true, true, true);
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotOperationalEulerFromNRT(
                    state, map, entry.second)));
        continue;
      }

    }
  }
}

void mwoibn::robot_class::RobotXBotRT::_loadControllers(
    YAML::Node config, XBot::SharedMemory::Ptr shared_memory)
{

  RobotXBotFeedback::_loadControllers(config);

  for (auto entry : config)
  {
    entry.second["name"] = entry.first.as<std::string>();

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["layer"].as<std::string>() == "RT")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::XBotControllerShared(
                  command, map,  entry.second, shared_memory)));
      continue;
    }
    if (entry.second["layer"].as<std::string>() == "NRT")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::XBotControllerToNRT(
                  command, map,  entry.second)));
      continue;
    }
  }
}
