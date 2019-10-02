#ifndef __MGNSS__PLUGINS__GENERATOR_H
#define __MGNSS__PLUGINS__GENERATOR_H

#include "mgnss/modules/base.h"
#include <mwoibn/std_utils/map.h>
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>



namespace mgnss
{
namespace plugins
{

//! This class provides a common interface for the ROS and XBotCore plugins 
/**
 * These plugins provide the middleware layer to the basic modules implementing an algorithm 
 * (e.g. a controller, state estimation, a communication plugin)
 */
template<typename Subscriber, typename Service, typename Node, typename Publisher>
class Generator
{
  typedef  std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> robot_map;


public:


//! Basic constructor take the plugin name
Generator(std::string name): name(name) { }


virtual ~Generator(){
}

//! Function executed, when the plugin is closed
/**
 * Executes the close function of the base modules to ensure a safe exit from the control loop, saves the logged data and closes the log file.
 */

virtual bool close(){
        controller_ptr->close(); // close the plugin
        logger_ptr->flush(); // flush the logged date from the memory for RT applications
        logger_ptr->close(); // close the log files
        return true;
}

//! Function executed, when the plugin is turned on
/**
 * Saves the initialization time, initalizes all the robot feedbacks, updates robot kinematics and sets initial conditions for the module
 */
virtual void start(double time)
{
        _start = time; // save the initialization time

        for(auto& robot: _robot_ptr) 
        _valid = robot.second->get() && robot.second->feedbacks.reset() && _valid; // check that all the feedbacks have been initializaed and update the state of the robot


        _rate = true; // rate is set before

        if (_valid) // if all the feedbacks are valid update the robot kinematics, set the initial condition in the module and change the plugin status to initialized
        {
          for(auto& robot: _robot_ptr) robot.second->updateKinematics();
                controller_ptr->init();
                _initialized = true;

        }

}


//! Function executed, when the plugin is stopped
/**
 * Executes the stop function of the base modules.
 */
virtual void stop(){
        controller_ptr->stop();
}

//! Function executed at each control step
virtual void control_loop(double time)
{

  _valid = true; // reset the flag

  for(auto& robot: _robot_ptr) //
        _valid = robot.second->get() && _valid; // check feedbacks status and read the feedbacks

  if (!_valid) // if a feddback reported error abandon the update loop
                return;

  for(auto& robot: _robot_ptr) robot.second->updateKinematics(); // update the robot kinematics

        if (!_initialized) // because, in xbotcore the start() method in invoked only once, check that the plugin was initialized correctly. If not, try to initialized it now
        {
                if(_valid) // if all feedback are initialized, initialized the plugin 
                        controller_ptr->init();

                if(_rate && _valid)
                        _initialized = true;
        }
        controller_ptr->update(); // update the plugin
        controller_ptr->send(); // set the desired robot state and send the plugin outputs
        controller_ptr->log(*logger_ptr.get(), time-_start); // log the data

        logger_ptr->write(); // write the data  - saves the data to file in NRT plugin and does nothing in RT
        _robot_ptr.begin()->second->wait(); // release the thread

}

//! Provides an access to the robot models
/**
 * This method is used in the shared plugin to share the robot models between the plugins
 */
robot_map& shareRobots(){return _robot_ptr;};
//! Moves the controller 
/**
 * This method is used in the shared plugin to move the controller to the shared space
 */
std::unique_ptr<mgnss::modules::Base> releaseController(){return std::move(controller_ptr);}

virtual std::vector<std::string> readRobots(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
  return {readRobot(config_file, secondary_file, config, plugin_config)};
}

//! Reads the confguration of the robot model
/**
 * @param[in] config_file - path to the main config file
 * @param[in] secondary_file - path to the secondary config file that overwrites the element of the main config file, if there is no secondary file pass an empty string
 * @param[in] config - an empty YAML::Node for the full configuration
 * @param[in] plugin_config - plugin configuration
 * @param[out] id - returns the robot id
 */
virtual std::string readRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
  config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

  config["robot"]["layer"] = plugin_config["layer"]; // pass layer parameter to the robot config
  config["robot"]["mode"] = plugin_config["mode"]; // pass plugin mode parameter to the robot config

  return config_file + "__" + plugin_config["robot"].as<std::string>() + "__" + secondary_file;

}



//! Reads the confguration file
/**
 * @param[in] config_file - path to the main config file
 * @param[in] config - an empty YAML::Node to read the full configuration
 * @param[in] plugin_config - an empty YAML::Node to read the plugin configuration
 * @param[out] return the path to the secondaru file
 */
std::string readConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config){
  _loadConfig(config_file, config, plugin_config); // reads the full configuration and plugin configuration
  return _checkConfig(plugin_config, config_file); // checks if all the required parameters for the plugin have been defined in the config file
}

//! Generate the instance of the module, initialize the communication layer
/**
 * @param[in] config - YAML::Node to read the configs
 * @param[in] plugin_config - a plugin configuration
 */
virtual void initModule(YAML::Node config, YAML::Node plugin_config){
  config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>()); // gat the full configuration
  config = config["modules"][name]; // get the plugin configuration
  config["name"] = name; // set the configuration name

  _resetPrt(config); // Create the instance of the module,
  _initCallbacks(config); // initialize the communication

  controller_ptr->kinematics.set(config["kinematics"].as<bool>()); // set whether the plugin needs the robot kinematic model
  controller_ptr->dynamics.set(config["dynamics"].as<bool>()); // set whether the plugin needs the robot dynamics
  controller_ptr->modify.set(config["model_change"].as<bool>()); // set whether the plugin modifes the robot state

  // initialize robot models

  for(auto& robot: _robot_ptr){
    robot.second->get();
    robot.second->updateKinematics();
  }
}

//! Reads the confguration file in the shared space
/**
 * @param[in] config  - read the full configuration
 * @param[in] plugin_config - the plugin configuration
 * @param[in] share - reference to the shared space
 */
virtual void initModule(YAML::Node config, YAML::Node plugin_config, mwoibn::communication_modules::Shared& share){
  config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
  config = config["modules"][name];
  config["name"] = name;

  _resetPrt(config); // Create the instance of the module
  _initCallbacks(config);  // initialize the communication
  _initCallbacks(config, share); // initialize the communication in the shared space

  controller_ptr->kinematics.set(config["kinematics"].as<bool>()); // set whether the plugin needs the robot kinematic model
  controller_ptr->dynamics.set(config["dynamics"].as<bool>()); // set whether the plugin needs the robot dynamics
  controller_ptr->modify.set(config["model_change"].as<bool>()); // set whether the plugin modifes the robot state

  // initialize robot models

  for(auto& robot: _robot_ptr){
    robot.second->get();
    robot.second->updateKinematics();
  }

}

std::shared_ptr<mwoibn::common::Logger> logger_ptr; // holds the pointer to the logger
std::unique_ptr<mgnss::modules::Base> controller_ptr; // holds the pointer to the module

std::shared_ptr<Node> n; // holds the ros node handler
std::string name = ""; // plugin name

protected:


std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot> > _robot_ptr; // keeps the map of robot models

//! Reads the main configuration file, and plugin configuration
virtual void _loadConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config){
  config = mwoibn::robot_class::Robot::getConfig(config_file);
  plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);
}

//! Checks if all the necessary parameters have been defined in the configuration files
virtual std::string _checkConfig(YAML::Node plugin_config, std::string config_file)
{
        if (!plugin_config["modules"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Could not find modules configuration."));
        if (!plugin_config["modules"][name])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Could not find ") + name + std::string(" module configuration."));

        plugin_config = plugin_config["modules"][name];

        if (!plugin_config["robot"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Could not find robot configuration in module parameters."));
        if (!plugin_config["layer"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify robot layer."));
        if (!plugin_config["mode"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify controller mode."));
        if (!plugin_config["controller"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify controllers to be loaded ."));
        if (!plugin_config["kinematics"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify the kinematics update."));
        if (!plugin_config["dynamics"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify the dynamics update."));
        if (!plugin_config["model_change"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Please specify if the robot model changes."));


        std::string secondary_file = "";
        if (plugin_config["secondary_file"])
                secondary_file = mwoibn::robot_class::Robot::readPath(plugin_config["secondary_file"]);

        std::cout << "secondary file " << secondary_file << std::endl;

        return secondary_file;

}

//! Function to change the plugin update rate
virtual void _setRate(double period){
        controller_ptr->setRate(period);
}
//! Function to call the module constructor
virtual void _resetPrt(YAML::Node config) = 0;

//! Function that initializes  the communication layer
virtual void _initCallbacks(YAML::Node config) = 0;
//! Function that initializes shared communication
virtual void _initCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& share){}

std::vector<Subscriber> _sub; // a placeholder for all the NRT subscribers defined for the plugin
std::vector<Service> _srv; // a placeholder for all the NRT services defined for the plugin





bool _initialized = false, _valid = false, _rate = false; // initialize all the flags
double _start; // time the plugin was started

};
}
}
#endif // __MGNSS__PLUGINS__GENERATOR_H
