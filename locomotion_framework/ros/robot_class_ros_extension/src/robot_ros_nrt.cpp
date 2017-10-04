#include "mwoibn/robot_class/robot_ros_nrt.h"

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(std::string config_file,
                                              std::string config_name,
                                              std::string secondary_file)
    : mwoibn::robot_class::RobotRos()
{

  YAML::Node config = _getConfig(
      config_file, secondary_file); // this is done twice with this robot

  YAML::Node robot = mwoibn::robot_class::RobotRos::_init(config, config_name);

  if (!robot["rate"])
    throw(std::invalid_argument(
        "Desired frequency not defined in a configuration " + config_file +
        ", " + config_name));

  _rate_ptr.reset(new ros::Rate(robot["rate"].as<double>()));

  try
  {
    _loadMappings(robot["mapping"]);
    _loadFeedbacks(robot["feedback"]);
    _loadControllers(robot["controller"]);
  }
  catch (const std::invalid_argument& e)
  {
    throw(std::invalid_argument("\nconfig_file:\t" + config_file +
                                std::string("\nrobot:\t") + config_name +
                                std::string("\n") + e.what()));
  }
}

void mwoibn::robot_class::RobotRosNRT::_loadFeedbacks(YAML::Node config)
{
  for (auto entry : config)
  {
    if (!_loadFeedback(entry.second, entry.first.as<std::string>()))
      continue;

    BiMap map = readBiMap(entry.second["dofs"]);

    if (entry.second["space"].as<std::string>() == "JOINT")
    {
      if(loadJointSpaceFeedback(entry.second, feedbacks, state, map))
        continue;
    }
    if (entry.second["space"].as<std::string>() == "OPERATIONAL")
    {
      _getDefaultPosition(entry.second, true, true, true);
      if(loadOperationalSpaceFeedback(entry.second, feedbacks, state, map))
      continue;
    }
  }
}

bool mwoibn::robot_class::RobotRosNRT::loadJointSpaceFeedback(
    YAML::Node config, Feedbacks& external_feedbacks, State& external_state,
    BiMap external_map)
{
  if (config["message"].as<std::string>() == "sensor_msgs::JointState")
  {
    external_feedbacks.add(
        std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
            new mwoibn::communication_modules::RosFeedback<
                sensor_msgs::JointState, sensor_msgs::JointState::ConstPtr>(
                external_state, external_map, config)));

    return true;
  }
  if (config["message"].as<std::string>() == "custom_messages::CustomCmnd")
  {
    external_feedbacks.add(
        std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
            new mwoibn::communication_modules::RosFeedback<
                custom_messages::CustomCmnd,
                custom_messages::CustomCmnd::ConstPtr>(external_state,
                                                       external_map, config)));

    return true;
  }
  return false;
}

bool mwoibn::robot_class::RobotRosNRT::loadOperationalSpaceFeedback(
    YAML::Node config, Feedbacks& external_feedbacks, State& external_state,
    BiMap external_map)
{
  if (config["message"].as<std::string>() == "gazebo_msgs::LinkStates")
  {

    external_feedbacks.add(
        std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
            new mwoibn::communication_modules::RosOperationalEuler<
                gazebo_msgs::LinkStates, gazebo_msgs::LinkStates::ConstPtr>(
                external_state, external_map, config)));
    return true;
  }

  return false;
}

bool mwoibn::robot_class::RobotRosNRT::loadRosControllers(
    YAML::Node config, Controllers& external_controllers, State& external_state,
    BiMap external_map)
{

  if (config["type"].as<std::string>() == "reference")
  {
    external_controllers.add(
        std::unique_ptr<mwoibn::communication_modules::BasicController>(
            new mwoibn::communication_modules::RosController(
                external_state, external_map, config)));
    return true;
  }
  return false;
}

void mwoibn::robot_class::RobotRosNRT::_loadMappings(YAML::Node config)
{

  for (auto entry : config)
  {
    if (biMaps().isDefined(entry.first.as<std::string>()))
    {
      std::cout << "Map " + entry.first.as<std::string>() +
                       " has been previously initilized, skip this map"
                << std::endl;
      continue;
    }

    entry.second["name"] = entry.first.as<std::string>();
    _loadMap(entry.second);
  }
}
void mwoibn::robot_class::RobotRosNRT::_loadMap(YAML::Node config)
{

  // link side state
  if (!config["source"])
    throw(std::invalid_argument("Define mapping source"));

  std::string topic = config["source"].as<std::string>();

  if (!config["name"])
    throw(std::invalid_argument("Please define a mapping name"));

  if (!config["message"])
    throw(std::invalid_argument("Please defined a message type."));

  std::string message = config["message"].as<std::string>();
  // if mappings has not been yet initialized try to do this from ROS base on a
  // message
  if (biMaps().isDefined(config["name"].as<std::string>()))
    return;

  if (message == "sensor_msgs::JointState")
  {
    if (!initMapping<sensor_msgs::JointState,
                     sensor_msgs::JointState::ConstPtr>(
            topic, config["name"].as<std::string>(), 10))
    {
      std::stringstream err_msg;
      err_msg << "Mapping " << config["name"].as<std::string>()
              << " has not been defined. Initializing from "
              << "ROS failed. No callback from topic " << topic
              << " was received in " << 10 << " tries." << std::endl;
      throw(std::invalid_argument(err_msg.str().c_str()));
    }
  }

  std::cout << "Map " << config["name"].as<std::string>()
            << " has been sucesfully loaded." << std::endl;
}

void mwoibn::robot_class::RobotRosNRT::_loadControllers(YAML::Node config)
{

  for (auto entry : config)
  {
    entry.second["name"] = entry.first.as<std::string>();

    BiMap map = readBiMap(entry.second["dofs"]);

    if (!entry.second["type"])
      throw(std::invalid_argument(std::string("Unknown controller type for " +
                                              entry.first.as<std::string>())));

    if (entry.second["type"].as<std::string>() ==
        "custom_controller/ActuatorPositionControllerClasses")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::CustomController(
                  command, map, entry.second)));
      continue;
    }

    loadRosControllers(entry.second, controllers, command, map);

  }
}

void mwoibn::robot_class::RobotRosNRT::_loadContacts(YAML::Node contacts_config)
{

  for (int i = 0; i < contacts_config.size() - 1; i++)
  {
    YAML::Node contact = contacts_config["contact" + std::to_string(i)];

    //    if (contact.first.as<std::string>() == "settings")
    //      continue;

    contact["settings"] = contacts_config["settings"];

    if (!contact["type"])
      throw std::invalid_argument(
          std::string("Please specify a contact type for ") +
          contact["name"].as<std::string>());
    try
    {
      //      contact.second["name"] = contact.first.as<std::string>();
      std::string type = contact["type"].as<std::string>();

      if (type.compare("point_foot") == 0)
      {
        _contacts->add(std::unique_ptr<ContactV2>(new ContactRos<ContactV2>(
            ContactV2(_model, state.state(INTERFACE::POSITION), contact),
            contact)));
        continue;
      }
      if (type.compare("wheel") == 0)
      {
        _contacts->add(
            std::unique_ptr<ContactV2>(new ContactRos<WheelContactV2>(
                WheelContactV2(_model, state.state(INTERFACE::POSITION),
                               contact),
                contact)));
        continue;
      }
      if (type.compare("wheel_locked") == 0)
      {
        _contacts->add(std::unique_ptr<ContactV2>(new ContactRos<WheelContact>(
            WheelContact(_model, state.state(INTERFACE::POSITION), contact),
            contact)));
        continue;
      }

      throw std::invalid_argument(std::string("Uknown contacy type: ") + type);
    }
    catch (std::invalid_argument& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}
