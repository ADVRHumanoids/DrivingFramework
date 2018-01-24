#include "mwoibn/robot_class/robot_ros_nrt.h"

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(std::string config_file,
                                              std::string config_name,
                                              std::string secondary_file)
    : mwoibn::robot_class::RobotRos()
{

  YAML::Node config = getConfig(
      config_file, secondary_file); // this is done twice with this robot

  YAML::Node robot;
  try
  {
    robot = mwoibn::robot_class::RobotRos::_init(config, config_name);
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

  wait();
  get();
  updateKinematics();
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
                external_state, external_map, config)), config["name"].as<std::string>());

    return true;
  }
  if (config["message"].as<std::string>() == "custom_messages::CustomCmnd")
  {
    external_feedbacks.add(
        std::unique_ptr<mwoibn::communication_modules::BasicFeedback>(
            new mwoibn::communication_modules::RosFeedback<
                custom_messages::CustomCmnd,
                custom_messages::CustomCmnd::ConstPtr>(external_state,
                                                       external_map, config)), config["name"].as<std::string>());

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
                external_state, external_map, config)), config["name"].as<std::string>());
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
                external_state, external_map, config)), config["name"].as<std::string>());
    return true;
  }
  return false;
}

void mwoibn::robot_class::RobotRosNRT::_loadMap(YAML::Node config)
{

  if (!config["name"])
    throw(std::invalid_argument("Please define a mapping name"));

  if (biMaps().isDefined(config["name"].as<std::string>()))
    return;

  if (!config["loading"])
    throw(std::invalid_argument("Please defined a mapping loading method."));

  if(config["loading"].as<std::string>() == "topic") _loadMapFromTopic(config);
  else if(config["loading"].as<std::string>() == "model") _loadMapFromModel(config);
  else throw(std::invalid_argument("Unknown loading method for mapping " + config["name"].as<std::string>()));

  std::cout << "Map " << config["name"].as<std::string>()
            << " has been sucesfully loaded." << std::endl;
}

void mwoibn::robot_class::RobotRosNRT::_loadMapFromTopic(YAML::Node config){

  if (!config["message"])
    throw(std::invalid_argument("Please defined a message type."));

  std::string message = config["message"].as<std::string>();

  if (!config["source"])
    throw(std::invalid_argument("Please defined a source."));

  std::string topic = config["source"].as<std::string>();

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

#ifdef ROS_CONTROL
    if (entry.second["type"].as<std::string>() ==
        "custom_controller/ActuatorPositionControllerClasses")
    {

      mwoibn::communication_modules::BasicFeedback* feedback = nullptr;

     // if (entry.second["initialize"] && entry.second["initialize"].as<std::string>() != "")
        feedback = &feedbacks.feedback(entry.second["initialize"].as<std::string>());

        if(feedback != nullptr){

          std::vector<std::string> controller_map = biMaps().get(entry.second["dofs"]["mapping"].as<std::string>()).getNames();
          std::vector<std::string> feedback_map = biMaps().get(feedback->getMapName()).getNames(); // HARDCODED
          mwoibn::VectorInt common;
          common.setConstant(controller_map.size(), mwoibn::NON_EXISTING);
          for(int i = 0; i < controller_map.size(); i++){
            for(int j = 0; j < feedback_map.size(); j++){
              if(controller_map[i] == feedback_map[j])
                common[i] = j;
              continue;
            }
          }
          BiMap temp_map("temp", common);
          controllers.add(
              std::unique_ptr<mwoibn::communication_modules::BasicController>(
                  new mwoibn::communication_modules::CustomController(
                      command, map, entry.second, feedback, &temp_map)),  entry.first.as<std::string>());
        }
        else
          controllers.add(
              std::unique_ptr<mwoibn::communication_modules::BasicController>(
                  new mwoibn::communication_modules::CustomController(
                      command, map, entry.second, feedback)),  entry.first.as<std::string>());
      continue;
    }
    if (entry.second["type"].as<std::string>() ==
        "custom_controller/ActuatorVelocityController")
    {

      controllers.add(
          std::unique_ptr<mwoibn::communication_modules::BasicController>(
              new mwoibn::communication_modules::VelocityController(
                  command, map, entry.second)),  entry.first.as<std::string>());
      continue;
    }
#endif
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
