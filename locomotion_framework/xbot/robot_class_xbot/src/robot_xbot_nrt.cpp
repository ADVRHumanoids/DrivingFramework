#include "mwoibn/robot_class/robot_xbot_nrt.h"
//#include "mwoibn/robot_class/robot_ros_nrt.h"

mwoibn::robot_class::RobotXBotNRT::RobotXBotNRT(std::string config_file,
                                                std::string config_name, std::string controller_source,
                                                std::string secondary_file)
    : RobotXBotFeedback()
{
  YAML::Node config = getConfig(
      config_file, secondary_file); // this is done twice with this robot

  YAML::Node middleware = config["ros"];

  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);
    if (!robot["rate"])
      throw(std::invalid_argument(
          "Desired frequency not defined in a configuration " + config_file +
          ", " + config_name));
    _rate_ptr.reset(new ros::Rate(robot["rate"].as<double>()));

    _sense = true; // because of the weird XBot behaviour in the NRT layer

    _loadConfig(middleware["feedback"], robot["feedback"]);
    _loadConfig(middleware["controller"], robot["controller"]);

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


mwoibn::robot_class::RobotXBotNRT::RobotXBotNRT(YAML::Node full_config,
                                                std::string config_name, std::string controller_source)
    : RobotXBotFeedback()
{
  YAML::Node config = YAML::Clone(full_config);
  YAML::Node middleware = config["ros"];

  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);
    if (!robot["rate"])
      throw(std::invalid_argument(
          std::string("Desired frequency not defined in a configuration ") +
          std::string(", ") + config_name));
    _rate_ptr.reset(new ros::Rate(robot["rate"].as<double>()));

    _sense = true; // because of the weird XBot behaviour in the NRT layer

    _loadConfig(middleware["feedback"], robot["feedback"]);
    _loadConfig(middleware["controller"], robot["controller"]);

    _init(config, robot);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument(std::string("\n robot configuration: \t ") +
                                config_name + std::string("\nerror:\t") +
                                e.what()));
  }
}

void mwoibn::robot_class::RobotXBotNRT::RobotXBotNRT::_loadFeedbacks(
    YAML::Node config)
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
    if(feedbacks.has(entry.first.as<std::string>())) continue;
    if(feedbacks.has("shared_"+entry.first.as<std::string>())) continue;

    if (entry.second["layer"].as<std::string>() == "NRT")
    {
      if (entry.second["space"].as<std::string>() == "JOINT")
      {
        entry.second["rate"] = rate();

        if(RobotRosNRT::loadJointSpaceFeedback(entry.second, feedbacks, state, map)){
         _spin = true;
        }
        continue;
      }
      if (entry.second["space"].as<std::string>() == "OPERATIONAL")
      {

        _getDefaultPosition(entry.second, true, true, true);

       if(RobotRosNRT::loadOperationalSpaceFeedback(entry.second, feedbacks, state, map)){
          _spin = true;
       }
         continue;

      }
    }
    if (entry.second["layer"].as<std::string>() == "RT")
    {
      if (entry.second["space"].as<std::string>() == "JOINT")
      {
        feedbacks.add(
            std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
                new mwoibn::communication_modules::XBotFeedbackFromRT(
                    state, map, entry.second)), entry.first.as<std::string>());
      }
    }
  }

  wait();
  get();
  updateKinematics();

}



void mwoibn::robot_class::RobotXBotNRT::RobotXBotNRT::_init(YAML::Node config, YAML::Node robot){
  if (!config["source"])
    throw(std::invalid_argument("Please define sources for XBot.\n"));

  if (!config["source"]["config"])
    throw(std::invalid_argument(
      "Please define path to XBot configuration file for feedback.\n"));

  if (!config["source"]["config"]["file"])
    throw(std::invalid_argument(
      "Please define path to XBot configuration file for feedback.\n"));

  std::string file = "";
  if (config["source"]["config"]["path"]) file = config["source"]["config"]["path"].as<std::string>();

  file += config["source"]["config"]["file"].as<std::string>();

  _robot = XBot::RobotInterface::getRobot(file);

  mwoibn::robot_class::RobotXBotFeedback::_init(config, robot);
}

void mwoibn::robot_class::RobotXBotNRT::RobotXBotNRT::_loadControllers(
    YAML::Node config)
{

  RobotXBotFeedback::_loadControllers(config);

  for (auto entry : config)
  {
    if (entry.first.as<std::string>() == "mode") continue;
    if (entry.first.as<std::string>() == "gains") continue;
    if (entry.first.as<std::string>() == "source") continue;

    entry.second["name"] = entry.first.as<std::string>();

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["layer"].as<std::string>() == "NRT")
    {
      entry.second["rate"] = rate();

      if(RobotRosNRT::loadRosControllers(entry.second, controllers, command, map))
      {
         _spin = true;
        continue;
      }
    }
    if (entry.second["layer"].as<std::string>() == "RT")
    {
      controllers.add(
            std::unique_ptr<mwoibn::communication_modules::BasicController>(
                new mwoibn::communication_modules::XBotControllerToRT(
                    command, map,  entry.second)), entry.first.as<std::string>());
        continue;
    }
  }
}
