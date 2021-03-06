#include "mwoibn/robot_class/robot_xbot_rt.h"
#include "mwoibn/communication_modules/communication_base.h"
#include "mwoibn/communication_modules/xbot_point_get.h"

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(
    XBot::RobotInterface::Ptr robot, std::string config_file,
    std::string config_name,
    std::string controller_source,
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
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

    if(!robot["rate"])
      throw(std::invalid_argument(
          std::string("Desired frequency not defined in the configuration ") +
          std::string(", ") + config_name));
    else
      std::cout << "rate\t" << robot["rate"].as<double>() << std::endl;

    setRate(1/robot["rate"].as<double>());
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
    std::string config_name, std::string controller_source,
    XBot::SharedMemory::Ptr shared_memory)
    : RobotXBotFeedback()
{
  _robot = robot;
  YAML::Node config = YAML::Clone(full_config);
  try
  {
    YAML::Node robot =
        mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

    if(!robot["rate"])
      throw(std::invalid_argument(
          std::string("Desired frequency not defined in the configuration ") +
          std::string(", ") + config_name));
    else
      std::cout << "rate\t" << robot["rate"].as<double>() << std::endl;

    setRate(1/robot["rate"].as<double>());

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


mwoibn::robot_class::RobotXBotRT::RobotXBotRT(XBot::RobotInterface::Ptr robot, std::string config_file,
            std::string config_name, mwoibn::communication_modules::Shared& shared,
            std::string controller_source,
            std::string secondary_file, XBot::SharedMemory::Ptr shared_memory){
            _robot = robot;

            YAML::Node config = getConfig(config_file,
                                secondary_file); // this is done twice with this robot

            try
            {
                YAML::Node robot =
                mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

                if(!robot["rate"])
                throw(std::invalid_argument(
                    std::string("Desired frequency not defined in the configuration ") +
                    std::string(", ") + config_name));
                else
                    std::cout << "rate\t" << robot["rate"].as<double>() << std::endl;

                setRate(1/robot["rate"].as<double>());
                    //    _init(config, robot);

                _init(config, robot, shared, shared_memory);
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

mwoibn::robot_class::RobotXBotRT::RobotXBotRT(XBot::RobotInterface::Ptr robot, YAML::Node full_config,
            std::string config_name, mwoibn::communication_modules::Shared& shared,
            std::string controller_source, XBot::SharedMemory::Ptr shared_memory){
            _robot = robot;
            YAML::Node config = YAML::Clone(full_config);
            try
            {
                YAML::Node robot =
                    mwoibn::robot_class::RobotXBot::_init(config, config_name, controller_source);

                if(!robot["rate"])
                  throw(std::invalid_argument(
                      std::string("Desired frequency not defined in the configuration ") +
                      std::string(", ") + config_name));
                else
                  std::cout << "rate\t" << robot["rate"].as<double>() << std::endl;

                setRate(1/robot["rate"].as<double>());

                _init(config, robot, shared, shared_memory);
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
  //  _xbot_map = biMaps().getId("XBOT")
  // _initContactsCallbacks(robot["contacts"], shared_memory);
  _loadMappings(robot["mapping"]);

  _loadFeedbacks(robot["feedback"], shared_memory);
  _loadControllers(robot["controller"], shared_memory);

  mwoibn::robot_class::RobotXBotFeedback::_initStates();
}

void mwoibn::robot_class::RobotXBotRT::_init(
    YAML::Node config, YAML::Node robot, mwoibn::communication_modules::Shared& shared, XBot::SharedMemory::Ptr shared_memory)
{

  biMaps().add(makeBiMap(getLinks(_robot->getEnabledJointNames()), "XBOT"));
  //  _xbot_map = biMaps().getId("XBOT")
  // _initContactsCallbacks(robot["contacts"], shared_memory, shared);
  _loadMappings(robot["mapping"]);
  _shareFeedbacks(robot["feedback"], shared);
  _loadFeedbacks(robot["feedback"], shared_memory);
  _loadControllers(robot["controller"], shared_memory);
  _shareControllers(robot["controller"], shared);

  mwoibn::robot_class::RobotXBotFeedback::_initStates();
}


void mwoibn::robot_class::RobotXBotRT::loadControllers(std::string config_file, std::string config_name,
                       mwoibn::communication_modules::Shared& shared,
                       std::string controller_source, std::string secondary_file, XBot::SharedMemory::Ptr shared_memory){

   loadControllers(getConfig(
   config_file, secondary_file), config_name, shared, controller_source, shared_memory); // this is done twice with this robot
}


void mwoibn::robot_class::RobotXBotRT::loadControllers(YAML::Node full_config, std::string config_name,
                     mwoibn::communication_modules::Shared& shared,  std::string controller_source, XBot::SharedMemory::Ptr shared_memory){

    YAML::Node config = YAML::Clone(full_config);

    YAML::Node robot;
    try
    {

      robot = _readRobotConfig(full_config, config_name, controller_source);
      // read ROS specific configuration
      config = config["xbot"];
      _loadConfig(config["controller"], robot["controller"]);

      _loadControllers(robot["controller"], shared_memory);
      _shareControllers(robot["controller"], shared);

      }
      catch (const std::invalid_argument& e)
      {
        throw(std::invalid_argument(std::string("\nrobot:\t") + config_name +
              std::string("\n") + e.what()));
      }

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

    if(feedbacks.has(entry.first.as<std::string>())) continue;
    if(feedbacks.has("shared_"+entry.first.as<std::string>())) continue;


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

void mwoibn::robot_class::RobotXBotRT::_initContactsCallbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory){
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        for(auto entry: config) std::cout << entry.first << std::endl;

        for(auto& contact : *_contacts) {
                std::unique_ptr<mwoibn::communication_modules::CommunicationBase> callback = _generateContactCallback(*contact, config[contact->getName()], shared_memory);
                if (callback != nullptr)
                        feedbacks.add(std::move(callback), contact->getName());
                else
                        std::cout << __PRETTY_FUNCTION__ << std::string("Could not initialize callback for ") << contact->getName() << std::endl;

        }
}

void mwoibn::robot_class::RobotXBotRT::_initContactsCallbacks(YAML::Node config, XBot::SharedMemory::Ptr shared_memory, mwoibn::communication_modules::Shared& shared){
        for(auto& contact : *_contacts) {
                std::unique_ptr<mwoibn::communication_modules::CommunicationBase> callback;
                callback = std::move(_generateContactCallback(*contact, config[contact->getName()], shared));
                if (callback == nullptr)
                        callback = std::move(_generateContactCallback(*contact, config[contact->getName()], shared_memory));
                if (callback != nullptr)
                        feedbacks.add(std::move(callback), contact->getName());
                else
                        std::cout << __PRETTY_FUNCTION__ << std::string("Could not initialize callback for ") << contact->getName() << std::endl;
        }
}




std::unique_ptr<mwoibn::communication_modules::CommunicationBase> mwoibn::robot_class::RobotXBotRT::_generateContactCallback(mwoibn::robot_points::Contact& contact, YAML::Node config, XBot::SharedMemory::Ptr shared){

  if (!config["name"])
    throw std::invalid_argument(std::string("Contact shared feedback: missing 'name' argument."));

  return  std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(
                                 new mwoibn::communication_modules::XBotPointGet(config, contact.wrench(), shared));


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
