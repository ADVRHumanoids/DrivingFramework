#include "mwoibn/robot_class/robot_xbot_rt.h"

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(
    XBot::RobotInterface::Ptr robot, std::string config_file, std::string robot_reference,
    XBot::SharedMemory::Ptr shared_memory)
    : RobotXBotFeedback()
{

  std::string xbot_file = config_file;

  YAML::Node config = YAML::LoadFile(config_file);

  std::cout << "robot start" << std::endl;
  _robot = robot;

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

//  std::string file_path = "";
//  if(config["xbot"] && config["xbot"]["source"] && config["xbot"]["source"]["config"]){
//      if(config["xbot"]["source"]["config"]["path"])
//        file_path = config["xbot"]["source"]["config"]["path"].as<std::string>();
//      if(config["xbot"]["source"]["config"]["path"])
//        file_path += config["xbot"]["source"]["config"]["file"].as<std::string>();
//  }

//  if(file_path.compare(xbot_file)){
//    std::cout << "WARNING: Different XBot config file has been received from constructor and config file. Proceed with the constructor file.\n";
//    std::cout << "\tconstructor:\t" << xbot_file << "\n";
//    std::cout << "\tfile:\t" << file_path << std::endl;
//    }

//  config["xbot"]["source"]["config"]["path"] = "";
//  config["xbot"]["source"]["config"]["file"] = xbot_file;

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
      if (entry.second["space"].as<std::string>() == "OPERATIONAL")
      {
        _getDefaultPosition(entry.second, true, true, true);
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotOperationalEulerFromNRT(
                    state, map, entry.second)), entry.first.as<std::string>());
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
    if (entry.first.as<std::string>() == "source") continue;

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
