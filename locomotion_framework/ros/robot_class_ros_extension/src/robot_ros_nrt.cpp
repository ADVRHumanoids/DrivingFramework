#include "mwoibn/robot_class/robot_ros_nrt.h"

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(std::string config_file,
                                              std::string config_name,
                                              std::string controller_source,
                                              std::string secondary_file)
        : mwoibn::robot_class::RobotRos()
{
    try{
      _init(getConfig(config_file, secondary_file), config_name, controller_source); // this is done twice with this robot
    }
    catch (const std::invalid_argument& e)
    {
                throw(config_file + e.what());
    }
}

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(YAML::Node full_config,
                                              std::string config_name, std::string controller_source)
        : mwoibn::robot_class::RobotRos()
{
      _init(full_config, config_name, controller_source);
}

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(std::string config_file,
                                              std::string config_name,
                                              mwoibn::communication_modules::Shared& shared,
                                              std::string controller_source, std::string secondary_file)
        : mwoibn::robot_class::RobotRos()
{
  try{
      _init(getConfig(config_file, secondary_file), config_name, shared, controller_source); // this is done twice with this robot
     }
     catch (const std::invalid_argument& e)
     {
                throw(config_file + e.what());
    }
}

mwoibn::robot_class::RobotRosNRT::RobotRosNRT(YAML::Node full_config,
                                              std::string config_name, mwoibn::communication_modules::Shared& shared,
                                              std::string controller_source)
        : mwoibn::robot_class::RobotRos()
{
      _init(full_config, config_name, shared, controller_source);
}

void mwoibn::robot_class::RobotRosNRT::_init(YAML::Node full_config, std::string config_name, std::string controller_source){
  YAML::Node config = YAML::Clone(full_config);
  YAML::Node robot;
  try
  {
          robot = mwoibn::robot_class::RobotRos::_init(config, config_name, controller_source);
          _initContactsCallbacks(robot["contacts"]);
          _loadMappings(robot["mapping"]);
          _loadFeedbacks(robot["feedback"]);
          _loadControllers(robot["controller"]);
  }
  catch (const std::invalid_argument& e)
  {
          throw(std::invalid_argument(std::string("\nrobot:\t") + config_name +
                                      std::string("\n") + e.what()));
  }

  wait();
  get();
  updateKinematics();

}

void mwoibn::robot_class::RobotRosNRT::_init(YAML::Node full_config, std::string config_name, mwoibn::communication_modules::Shared& shared, std::string controller_source){
  YAML::Node config = YAML::Clone(full_config);
  YAML::Node robot;
  try
  {
          robot = mwoibn::robot_class::RobotRos::_init(config, config_name, controller_source);
          _initContactsCallbacks(robot["contacts"], shared);
          _loadMappings(robot["mapping"]);
          _shareFeedbacks(robot["feedback"], shared);
          _loadFeedbacks(robot["feedback"]);
          _loadControllers(robot["controller"]);
          _shareControllers(robot["controller"], shared);
  }
  catch (const std::invalid_argument& e)
  {
          throw(std::invalid_argument(std::string("\nrobot:\t") + config_name +
                                      std::string("\n") + e.what()));
  }

  wait();
  get();
  updateKinematics();

}

void mwoibn::robot_class::RobotRosNRT::loadControllers(YAML::Node full_config, std::string config_name,
                                                  mwoibn::communication_modules::Shared& shared,
                                                  std::string controller_source)
{
          YAML::Node config = YAML::Clone(full_config);

          YAML::Node robot;
          try
          {

              robot = _readRobotConfig(full_config, config_name, controller_source);
                // read ROS specific configuration
              config = config["ros"];
              _loadConfig(config["controller"], robot["controller"]);


              _loadControllers(robot["controller"]);
              _shareControllers(robot["controller"], shared);

            }
            catch (const std::invalid_argument& e)
            {
                        throw(std::invalid_argument(std::string("\nrobot:\t") + config_name +
                                                    std::string("\n") + e.what()));
            }

}


void mwoibn::robot_class::RobotRosNRT::loadControllers(std::string config_file,
                                              std::string config_name,
                                              mwoibn::communication_modules::Shared& shared,
                                              std::string controller_source, std::string secondary_file)
{
        loadControllers(getConfig(
                config_file, secondary_file), config_name, shared, controller_source); // this is done twice with this robot
}







void mwoibn::robot_class::RobotRosNRT::_loadFeedbacks(YAML::Node config)
{
        for (auto entry : config)
        {
                if (!_loadFeedback(entry.second, entry.first.as<std::string>()))
                        continue;

                BiMap map = readBiMap(entry.second["dofs"]);
                if(feedbacks.has(entry.second["name"].as<std::string>())) continue;

                if (entry.second["space"].as<std::string>() == "JOINT")
                {
                        if(!entry.second["function"])
                                throw(std::invalid_argument("Please defined the function of a joint space feedback" + entry.second["name"].as<std::string>()));
                        if(entry.second["function"].as<std::string>() != "state" && entry.second["function"].as<std::string>() != "reference" )
                                throw(std::invalid_argument("Unkown function of a joint space feedback " + entry.second["name"].as<std::string>() + std::string(": '") + entry.second["function"].as<std::string>() + std::string("' available options 'state' and 'reference'")));

                        entry.second["rate"] = rate();
                        if(entry.second["function"].as<std::string>() == "state" && loadJointSpaceFeedback(entry.second, feedbacks, state, map))
                                continue;
                        if(entry.second["function"].as<std::string>() == "reference" && loadJointSpaceFeedback(entry.second, feedbacks, command, map))
                                continue;
                }
                if (entry.second["space"].as<std::string>() == "OPERATIONAL")
                {
                        _getDefaultPosition(entry.second, true, true, true);
                        if(loadOperationalSpaceFeedback(entry.second, feedbacks, state, map))
                                continue;
                }
        }

        command.position.set(state.position.get());
}


bool mwoibn::robot_class::RobotRosNRT::loadJointSpaceFeedback(
        YAML::Node config, communication_modules::Communications& external_feedbacks, State& external_state,
        BiMap& external_map)
{
        if (config["message"].as<std::string>() == "sensor_msgs::JointState")
        {
                external_feedbacks.add( mwoibn::communication_modules::RosFeedback<
                                                sensor_msgs::JointState, sensor_msgs::JointState::ConstPtr>(
                                                external_state, external_map, config), config["name"].as<std::string>());

                return true;
        }
        if (config["message"].as<std::string>() == "custom_messages::CustomCmnd")
        {
                external_feedbacks.add( mwoibn::communication_modules::RosFeedback<
                                                custom_messages::CustomCmnd, custom_messages::CustomCmnd::ConstPtr>(external_state,
                                                                                                                    external_map, config), config["name"].as<std::string>());

                return true;
        }
        return false;
}

bool mwoibn::robot_class::RobotRosNRT::loadOperationalSpaceFeedback(
        YAML::Node config, communication_modules::Communications& external_feedbacks, State& external_state,
        BiMap& external_map)
{
        if (config["message"].as<std::string>() == "gazebo_msgs::LinkStates")
        {

                external_feedbacks.add(mwoibn::communication_modules::RosOperationalEuler<
                                               gazebo_msgs::LinkStates, gazebo_msgs::LinkStates::ConstPtr>(
                                               external_state, external_map, config), config["name"].as<std::string>());
                return true;
        }

        return false;
}

bool mwoibn::robot_class::RobotRosNRT::loadRosControllers(
        YAML::Node config, communication_modules::Communications& external_controllers, State& external_state,
        BiMap& external_map)
{

        if (config["type"].as<std::string>() == "reference")
        {
                external_controllers.add(mwoibn::communication_modules::RosController(external_state, external_map, config),
                                         config["name"].as<std::string>());
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
                            topic, config["name"].as<std::string>(), 100))
                {
                        std::stringstream err_msg;
                        err_msg << "Mapping " << config["name"].as<std::string>()
                                << " has not been defined. Initializing from "
                                << "ROS failed. No callback from topic " << topic
                                << " was received in " << 100 << " tries." << std::endl;
                        throw(std::invalid_argument(err_msg.str().c_str()));
                }
        }
}

void mwoibn::robot_class::RobotRosNRT::_loadControllers(YAML::Node config)
{

        for (auto entry : config)
        {
                if (entry.first.as<std::string>() == "mode") continue;

                entry.second["name"] = entry.first.as<std::string>();

                if(controllers.has(entry.first.as<std::string>())){
                        std::cout << __PRETTY_FUNCTION__ << std::string("Controller ") << entry.first.as<std::string>()
                                  << " has already been loaded in robot " << name() << std::endl;
                        continue;
                }

                BiMap map = readBiMap(entry.second["dofs"]);



                if (!entry.second["type"])
                        throw(std::invalid_argument(std::string("Unknown controller type for " +
                                                                entry.first.as<std::string>())));

#ifdef ROS_CONTROL
                if (entry.second["type"].as<std::string>() ==
                    "custom_controller/ActuatorPositionControllerClasses")
                {
                        mwoibn::communication_modules::CommunicationBase* feedback = nullptr;
                        if (config["mode"].as<std::string>() == "idle") {
                                std::cout << "Robot in the IDLE mode - lower-level controller " << entry.first.as<std::string>() <<
                                " has not been initialized." << std::endl;
                                continue;
                        }


                        controllers.add( mwoibn::communication_modules::CustomController(command, map,  entry.second), entry.first.as<std::string>());

                        continue;
                }
                if (entry.second["type"].as<std::string>() ==
                    "custom_controller/ActuatorVelocityController")
                {
                        if (config["mode"].as<std::string>() == "idle") {
                                std::cout << "Robot in the IDLE mode - lower-level controller " << entry.first.as<std::string>() <<
                                " has not been initialized." << std::endl;
                                continue;
                        }
                        controllers.add(
                                std::unique_ptr<mwoibn::communication_modules::BasicController>(
                                        new mwoibn::communication_modules::VelocityController(
                                                command, map, entry.second)),  entry.first.as<std::string>());
                        continue;
                }
#endif
                entry.second["rate"] = rate();

                loadRosControllers(entry.second, controllers, command, map);

        }
}

std::unique_ptr<mwoibn::communication_modules::CommunicationBase> mwoibn::robot_class::RobotRosNRT::_generateContactCallback(mwoibn::robot_points::Contact& contact, YAML::Node config){

  if(!config["topic"]) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;
  //if(!config["settings"]["feedback"].as<bool>()) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;

  if(!config["message"]) return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);;


  if(config["message"].as<std::string>() == "gazebo_msgs::ContactsState")
       return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(new mwoibn::communication_modules::RosContactSimulation(contact.wrench(), config));
  else if(config["message"].as<std::string>() == "geometry_msgs::Twist"){
       config["source"] = config["topic"];
    return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(new mwoibn::communication_modules::RosPointFeedback(contact.wrench(), config));
  }

   return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);

}

//
// void mwoibn::robot_class::RobotRosNRT::_loadContacts(YAML::Node contacts_config)
// {
//         YAML::Node loaded_contacts;
//
//         for (int i = 0; i < contacts_config.size() - 1; i++)
//         {
//                 YAML::Node contact = contacts_config["contact" + std::to_string(i)];
//                 //    if (contact.first.as<std::string>() == "settings")
//                 //      continue;
//                 contact["settings"] = contacts_config["settings"];
//
//                 if (!contact["type"])
//                         throw std::invalid_argument(
//                                       std::string("Please specify a contact type for ") +
//                                       contact["name"].as<std::string>());
//                 try
//                 {
//                         //      contact.second["name"] = contact.first.as<std::string>();
//                         std::string type = contact["type"].as<std::string>();
//
//                         if (type.compare("point_foot") == 0)
//                         {
//                                 _contacts->add(std::unique_ptr<robot_points::ContactV2>(new ContactRos<robot_points::ContactV2>(
//                                                                                                 robot_points::ContactV2(_model, state, contact), contact)));
//                                 loaded_contacts[_contacts->end()[-1]->getName()] = contact;
//                                 continue;
//                         }
//                         if (type.compare("wheel") == 0)
//                         {
//                                 _contacts->add(
//                                         std::unique_ptr<robot_points::ContactV2>(new ContactRos<robot_points::WheelContactV2>(
//                                                                                          robot_points::WheelContactV2(_model, state, contact), contact)));
//                                 loaded_contacts[_contacts->end()[-1]->getName()] = contact;
//                                 continue;
//                         }
//                         // if (type.compare("wheel_locked") == 0)
//                         // {
//                         //         _contacts->add(std::unique_ptr<ContactV2>(new ContactRos<WheelContact>(
//                         //                                                           WheelContact(_model, state.state("POSITION"), contact),
//                         //                                                           contact)));
//                         //         continue;
//                         // }
//
//                         throw std::invalid_argument(std::string("Uknown contacy type: ") + type);
//                 }
//                 catch (std::invalid_argument& e)
//                 {
//                         std::cout << e.what() << std::endl;
//                 }
//         }
//
//         _center_of_pressure->init(); // bacuse it needs contact feeback
//         contacts_config = loaded_contacts;
// }
