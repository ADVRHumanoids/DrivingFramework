#include "mwoibn/robot_class/robot_xbot_rt.h"

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(
    XBot::RobotInterface::Ptr robot, std::string config_file,
    std::string config_name,
    std::string secondary_file,
    XBot::SharedMemory::Ptr shared_memory)
    : RobotXBotFeedback()
{
  _robot = robot;

  YAML::Node config = getConfig(config_file,
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

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(
    XBot::RobotInterface::Ptr robot, YAML::Node full_config,
    std::string config_name,
    XBot::SharedMemory::Ptr shared_memory)
    : RobotXBotFeedback()
{
  _robot = robot;
  YAML::Node config = YAML::Clone(full_config);
  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name);
    //    _init(config, robot);

    _init(config, robot, shared_memory);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config file:\t" +
                                std::string("\n robot configuration: \t ") +
                                config_name + std::string("\nerror:\t") +
                                e.what()));
  }

  _sense = true;
  }

void mwoibn::robot_class::RobotXBotRT::_init(
    YAML::Node config, YAML::Node robot, XBot::SharedMemory::Ptr shared_memory)
{

  biMaps().add(makeBiMap(getLinks(_robot->getEnabledJointNames()), "XBOT"));
  //  _xbot_map = biMaps().getId("XBOT");
  _loadMappings(robot["mapping"]);

  _loadFeedbacks(robot["feedback"], shared_memory);
  _loadControllers(robot["controller"], shared_memory);

  mwoibn::robot_class::RobotXBotFeedback::_initStates();
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
                    state, map, entry.second, shared_memory)), entry.first.as<std::string>());
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
                    state, map, entry.second)), entry.first.as<std::string>());

        continue;
      }
      /*
      if (entry.second["space"].as<std::string>() == "OPERATIONAL")
      {
        _getDefaultPosition(entry.second, true, false, true);
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotOperationalEulerFromNRT(
                    state, map, entry.second)), entry.first.as<std::string>());
        continue;
      }
      */

    }
  }
}

void mwoibn::robot_class::RobotXBotRT::_loadControllers(
    YAML::Node config, XBot::SharedMemory::Ptr shared_memory)
{

  RobotXBotFeedback::_loadControllers(config);

  for (auto entry : config)
  {
    if (entry.first.as<std::string>() == "source") continue;
    if (entry.first.as<std::string>() == "mode") continue;
    if (entry.first.as<std::string>() == "gains") continue;


    entry.second["name"] = entry.first.as<std::string>();

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["layer"].as<std::string>() == "RT")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::XBotControllerShared(
                  command, map,  entry.second, shared_memory)), entry.first.as<std::string>());
      continue;
    }
    if (entry.second["layer"].as<std::string>() == "NRT")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::XBotControllerToNRT(
                  command, map,  entry.second)), entry.first.as<std::string>());
      continue;
    }
  }
}
