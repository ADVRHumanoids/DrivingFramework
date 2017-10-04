#include "mwoibn/robot_class/robot_xbot_feedback.h"

mwoibn::robot_class::RobotXBotFeedback::RobotXBotFeedback(
    std::string config_file, std::string config_name,
    std::string secondary_file)
    : RobotXBot()
{

  YAML::Node config = _getConfig(
      config_file, secondary_file); // this is done twice with this robot

  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name);

    _init(config, robot);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config file:\t" + config_file +
                                std::string("\n robot configuration: \t ") +
                                config_name + std::string("\nerror:\t") +
                                e.what()));
  }
}

void mwoibn::robot_class::RobotXBotFeedback::_init(YAML::Node config,
                                                   YAML::Node robot)
{

  if (!config["source"])
    throw(std::invalid_argument("Please define sources for XBot.\n"));

  if (!config["source"]["config"])
    throw(std::invalid_argument(
        "Please define path to XBot configuration file for feedback.\n"));

  _robot = XBot::RobotInterface::getRobot(
      config["source"]["config"].as<std::string>());

  biMaps().addMap(makeBiMap(getLinks(_robot->getEnabledJointNames()), "XBOT"));
  //  _xbot_map = biMaps().getId("XBOT");

  _loadFeedbacks(robot["feedback"]);
  _loadControllers(robot["controller"]);
}

void mwoibn::robot_class::RobotXBotFeedback::_loadFeedbacks(YAML::Node config)
{
  for (auto entry : config)
  {

    if (!_loadFeedback(entry.second, entry.first.as<std::string>()))
      continue;

    if (!entry.second["layer"])
      throw(std::invalid_argument("Please defined type of a feedack " +
                                  entry.first.as<std::string>()));

    if (entry.second["layer"].as<std::string>() != "online")
      continue;

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["space"].as<std::string>() == "JOINT")
    {
      feedbacks.add(
          std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
              new mwoibn::communication_modules::XBotFeedbackOnline(
                  state, map, entry.second, *_robot)));
      _sense = true;
      std::cout << "Loaded feedback " << entry.second["name"] << std::endl;
      continue;
    }

    if (entry.second["space"].as<std::string>() == "OPERATIONAL")
    {
      _getDefaultPosition(entry.second, false, true, true);

      feedbacks.add(
          std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
              new mwoibn::communication_modules::XBotOperationalEuler(
                  state, map, entry.second, *_robot)));

      _sense = true;

      continue;
    }
  }
}

void mwoibn::robot_class::RobotXBotFeedback::_loadControllers(YAML::Node config)
{

  for (auto entry : config)
  {

    if (!entry.second["layer"])
      throw(std::invalid_argument("Please defined type of a feedack " +
                                  entry.first.as<std::string>()));

    if (entry.second["layer"].as<std::string>() != "lower_level")
      continue;

    entry.second["name"] = entry.first.as<std::string>();

    BiMap map = readBiMap(entry.second["dofs"]);

    controllers.add(
        std::unique_ptr<mwoibn::communication_modules::BasicController>(
            new mwoibn::communication_modules::XBotLowerLevel(
                command, map, entry.second, *_robot)));

    _move = true;

    continue;
  }
}
