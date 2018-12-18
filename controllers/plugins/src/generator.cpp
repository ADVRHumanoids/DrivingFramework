#include "mgnss/plugins/ros_base.h"
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>


std::string mgnss::plugins::Generator::_checkConfig(YAML::Node plugin_config, std::string config_file)
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


        std::string secondary_file = "";
        if (plugin_config["secondary_file"])
                secondary_file = mwoibn::robot_class::Robot::readPath(plugin_config["secondary_file"]);

        std::cout << "secondary file " << secondary_file << std::endl;

        return secondary_file;

}

std::string mgnss::plugins::Generator::readRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
      config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

      config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
      config["robot"]["mode"] = plugin_config["mode"].as<std::string>();

      return config_file + "__" + plugin_config["robot"].as<std::string>() + "__" + secondary_file;

}

// void mgnss::plugins::RosBase::_loadRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
// }

void mgnss::plugins::Generator::initModule(YAML::Node config, YAML::Node plugin_config){
        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][name];
        config["name"] = name;
        _resetPrt(config);
        _initCallbacks(config);

        for(auto& robot: _robot_ptr){
          robot.second->get();
          robot.second->updateKinematics();
        }
}


void mgnss::plugins::Generator::_loadConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config)
{
        config = mwoibn::robot_class::Robot::getConfig(config_file);
        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);
}

std::string mgnss::plugins::Generator::readConfig(std::string config_file, YAML::Node& config, YAML::Node& plugin_config){
  _loadConfig(config_file, config, plugin_config);
  return _checkConfig(plugin_config, config_file);
}


void mgnss::plugins::Generator::initModule(YAML::Node config, YAML::Node plugin_config, mwoibn::communication_modules::Shared& share){
        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][name];
        config["name"] = name;

        _resetPrt(config);
        _initCallbacks(config);
        _initCallbacks(config, share);

        for(auto& robot: _robot_ptr){
          robot.second->get();
          robot.second->updateKinematics();
        }
}


void mgnss::plugins::Generator::start(double time)
{
        _start = time;

        for(auto& robot: _robot_ptr)  _valid = robot.second->get() && _valid;


        _rate = true;

        if (_valid)
        {
          for(auto& robot: _robot_ptr) robot.second->updateKinematics();
                controller_ptr->init();
                _initialized = true;

        }

}

void mgnss::plugins::Generator::stop() {
        controller_ptr->stop();
}

void mgnss::plugins::Generator::control_loop(double time)
{
  _valid = true;
  for(auto& robot: _robot_ptr)
    _valid = robot.second->get() && _valid;

        if (!_valid)
                return;

  for(auto& robot: _robot_ptr)
        robot.second->updateKinematics();

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

bool mgnss::plugins::Generator::close() {
        controller_ptr->close();
        logger_ptr->flush();
        logger_ptr->close();
        return true;
}
