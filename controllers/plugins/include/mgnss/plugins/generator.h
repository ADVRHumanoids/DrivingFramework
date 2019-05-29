#ifndef __MGNSS__PLUGINS__GENERATOR_H
#define __MGNSS__PLUGINS__GENERATOR_H

#include "mgnss/modules/base.h"
//#include <mwoibn/common/ros_logger.h>
//#include <ros/ros.h>
#include "mwoibn/std_utils/map.h"
//#include "mgnss/plugins/ros_base.h"
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>


// #define MGNSS_REGISTER_ROS_PLUGIN_(constructor) \
// extern "C" mgnss::plugins::RosBase* make() \
// { \
//   return new constructor(); \
// } \

namespace mgnss
{
namespace plugins
{

template<typename Subscriber, typename Service, typename Node>
class Generator
{
  typedef  std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> robot_map;
  typedef  MapKeyIterator<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _key_iter;
  typedef  MapValueIterator<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> _val_iter;

public:
// RosBase(int argc, char** argv, std::string name): _name(name){
//    _init(argc, argv);
// }

Generator(std::string name): name(name) { }

// virtual void connect(int argc, char** argv, std::string name);
// virtual void connect(std::string name);

virtual ~Generator(){
}

// virtual bool init(std::string name);
// virtual bool init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name);

virtual bool close(){
        controller_ptr->close();
        logger_ptr->flush();
        logger_ptr->close();
        return true;
}

virtual void start(double time)
{
        _start = time;

        for(auto& robot: _robot_ptr)  //_valid = robot.second->get() && _valid;
        _valid = robot.second->get() && robot.second->feedbacks.reset() && _valid;


        _rate = true;

        if (_valid)
        {
          for(auto& robot: _robot_ptr) robot.second->updateKinematics();
                controller_ptr->init();
                _initialized = true;

        }

}


virtual void stop(){
        controller_ptr->stop();
}

virtual void control_loop(double time)
{

  int i = 0;
  _valid = true;

  for(auto& robot: _robot_ptr)
    _valid = robot.second->get() && _valid;

        if (!_valid)
                return;

  //std::cout << "here!" << std::endl;

  for(auto& robot: _robot_ptr) robot.second->updateKinematics();

        if (!_initialized)
        {
//      if(!_rate){
                //_setRate(period); // here I may need a controller method
//      }
                if(_valid)
                        controller_ptr->init();

                if(_rate && _valid)
                        _initialized = true;
        }
        controller_ptr->update();
        controller_ptr->send();
        controller_ptr->log(*logger_ptr.get(), time-_start);

        logger_ptr->write();
        _robot_ptr.begin()->second->wait();

}

robot_map& shareRobots(){return _robot_ptr;};
std::unique_ptr<mgnss::modules::Base> releaseController(){return std::move(controller_ptr);}

virtual std::vector<std::string> readRobots(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
  return {readRobot(config_file, secondary_file, config, plugin_config)};
}

virtual std::string readRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
  config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

  config["robot"]["layer"] = plugin_config["layer"];//.as<std::string>();
  config["robot"]["mode"] = plugin_config["mode"];//.as<std::string>();

  return config_file + "__" + plugin_config["robot"].as<std::string>() + "__" + secondary_file;

}



std::string readConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config){
  _loadConfig(config_file, config, plugin_config);
  return _checkConfig(plugin_config, config_file);
}

virtual void initModule(YAML::Node config, YAML::Node plugin_config){
  config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
  config = config["modules"][name]; // here I should have the information about the module - I can add the modify, kinematic and dynamics info?
  config["name"] = name;

  _resetPrt(config);
  _initCallbacks(config);

  // std::cout << __PRETTY_FUNCTION__ << std::endl;
  // for(auto entry: config) std::cout << entry.first << std::endl;

  controller_ptr->kinematics.set(config["kinematics"].as<bool>());
  controller_ptr->dynamics.set(config["dynamics"].as<bool>());
  controller_ptr->modify.set(config["model_change"].as<bool>());

  for(auto& robot: _robot_ptr){
    robot.second->get();
    robot.second->updateKinematics();
  }
}

virtual void initModule(YAML::Node config, YAML::Node plugin_config, mwoibn::communication_modules::Shared& share){
  config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
  config = config["modules"][name];
  config["name"] = name;


    // std::cout << __PRETTY_FUNCTION__ << std::endl;
    // for(auto entry: config) std::cout << entry.first << std::endl;

  _resetPrt(config);
  _initCallbacks(config);
  _initCallbacks(config, share);

  controller_ptr->kinematics.set(config["kinematics"].as<bool>());
  controller_ptr->dynamics.set(config["dynamics"].as<bool>());
  controller_ptr->modify.set(config["model_change"].as<bool>());

  for(auto& robot: _robot_ptr){
    robot.second->get();
    robot.second->updateKinematics();
  }

}

std::shared_ptr<mwoibn::common::Logger> logger_ptr;
std::unique_ptr<mgnss::modules::Base> controller_ptr;

std::shared_ptr<Node> n;
std::string name = "";

protected:


std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot> > _robot_ptr;

virtual void _loadConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config){
  config = mwoibn::robot_class::Robot::getConfig(config_file);
  plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);
}

//virtual void _loadRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config);
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

//std::string _configFile();
virtual void _setRate(double period){
        controller_ptr->setRate(period);
}
virtual void _resetPrt(YAML::Node config) = 0;

virtual void _initCallbacks(YAML::Node config) = 0;
virtual void _initCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& share){}


// virtual void _init();

// std::vector<ros::Subscriber> _sub;
// std::vector<ros::ServiceServer> _srv;
std::vector<Subscriber> _sub;
std::vector<Service> _srv;



bool _initialized = false, _valid = false, _rate = false;
double _start;

};
}
}
#endif // RT_MY_TEST_H
