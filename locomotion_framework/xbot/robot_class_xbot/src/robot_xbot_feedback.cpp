#include "mwoibn/robot_class/robot_xbot_feedback.h"

mwoibn::robot_class::RobotXBotFeedback::RobotXBotFeedback(
    std::string config_file, std::string config_name,
    std::string controller_source, std::string secondary_file)
    : RobotXBot()
{

  YAML::Node config = getConfig(
      config_file, secondary_file); // this is done twice with this robot

  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

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

mwoibn::robot_class::RobotXBotFeedback::RobotXBotFeedback(
    YAML::Node full_config, std::string config_name, std::string controller_source)
    : RobotXBot()
{
  YAML::Node config = YAML::Clone(full_config);
  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

    _init(config, robot);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("config file:\t" +
                                std::string("\n robot configuration: \t ") +
                                config_name + std::string("\nerror:\t") +
                                e.what()));
  }
}

void mwoibn::robot_class::RobotXBotFeedback::_init(YAML::Node config,
                                                   YAML::Node robot)
{


  biMaps().add(makeBiMap(getLinks(_robot->getEnabledJointNames()), "XBOT"));

  _loadMappings(robot["mapping"]);
  _loadFeedbacks(robot["feedback"]);
  _loadControllers(robot["controller"]);

  _initStates();


}

void mwoibn::robot_class::RobotXBotFeedback::_initStates(){
    _robot->sense();

    mwoibn::VectorN reference(_robot->getJointNum());

    _robot->getMotorPosition(reference);
    _robot->setPositionReference(reference);

    reference.setZero();

    _robot->setVelocityReference(reference);

}

void mwoibn::robot_class::RobotXBotFeedback::_loadFeedbacks(YAML::Node config)
{
  for (auto entry : config)
  {

    if (!_loadFeedback(entry.second, entry.first.as<std::string>()))
      continue;

    if (!entry.second["layer"])
      throw(std::invalid_argument("Please defined type of a feedback " +
                                  entry.first.as<std::string>()));

    if (entry.second["layer"].as<std::string>() != "online")
      continue;

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["space"].as<std::string>() == "JOINT")
    {

      feedbacks.add(
          std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
              new mwoibn::communication_modules::XBotFeedbackOnline(
                  state, lower_limits, upper_limits, map, entry.second, *_robot)), entry.first.as<std::string>());
      _sense = true;
      std::cout << "Loaded feedback " << entry.second["name"] << std::endl;
      continue;
    }

    if (entry.second["space"].as<std::string>() == "OPERATIONAL")
    {

      _getDefaultPosition(entry.second, false, false, true);

      feedbacks.add(
          std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
              new mwoibn::communication_modules::XBotOperationalEuler(
                  state, map, entry.second, *_robot, rate())), entry.first.as<std::string>());

      _sense = true;

      continue;
    }
  }
}

void mwoibn::robot_class::RobotXBotFeedback::_loadControllers(YAML::Node config)
{

  std::cout << "load controllers" << std::endl;
  for (auto entry : config)
  {
    if (entry.first.as<std::string>() == "mode") continue;
    std::cout << entry.first << std::endl;
    if (entry.first.as<std::string>() == "gains") continue;
    if (entry.first.as<std::string>() == "source") continue;

    if (!entry.second["layer"])
      throw(std::invalid_argument("Please define controller type" +
                                  entry.first.as<std::string>()));

    if (entry.second["layer"].as<std::string>() != "lower_level")
      continue;

    if (!config["mode"])
      throw(std::invalid_argument("Please define robot operational mode " +
                                  entry.first.as<std::string>()));

    if (config["mode"].as<std::string>() == "idle"){
      std::cout << "Robot in the IDLE mode - lower-level controller " << entry.first.as<std::string>() <<
                   " has not been initialized." << std::endl;
      continue;
    }

    entry.second["name"] = entry.first.as<std::string>();
    entry.second["gains"] = config["gains"][entry.second["name"].as<std::string>()];

    BiMap map = readBiMap(entry.second["dofs"]);

    controllers.add(
        std::unique_ptr<mwoibn::communication_modules::BasicController>(
            new mwoibn::communication_modules::XBotLowerLevel(
                command, lower_limits, upper_limits, map, entry.second, *_robot)), entry.first.as<std::string>());

    _move = true;

    continue;
  }
}
