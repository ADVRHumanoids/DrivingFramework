#include "mgnss/plugins/ros_base.h"
#include <mwoibn/communication_modules/shared.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)


void mgnss::plugins::RosBase::connect(std::string name)
{
        _name = name;
        ros::init(std::map<std::string, std::string>(),getName());
        _n.reset(new ros::NodeHandle());
}

void mgnss::plugins::RosBase::connect(int argc, char** argv, std::string name)
{
        _name = name;
        ros::init(argc, argv, getName());
        _n.reset(new ros::NodeHandle());
        // init();
        // ros::NodeHandle n;
}

std::string mgnss::plugins::RosBase::_checkConfig(YAML::Node plugin_config, std::string config_file)
{
        if (!plugin_config["modules"])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Could not find modules configuration."));
        if (!plugin_config["modules"][_name])
                throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": ") +  config_file +
                                            std::string("\n\t Could not find ") + _name + std::string(" module configuration."));

        plugin_config = plugin_config["modules"][_name];

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

std::vector<std::string> mgnss::plugins::RosBase::readRobots(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
    return {_readRobot(config_file, secondary_file, config, plugin_config)};
}

std::string mgnss::plugins::RosBase::_readRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
      config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

      config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
      config["robot"]["mode"] = plugin_config["mode"].as<std::string>();

      return config_file + "__" + plugin_config["robot"].as<std::string>() + "__" + secondary_file;

}

void mgnss::plugins::RosBase::_loadRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
        std::string robot_id = _readRobot(config_file, secondary_file, config, plugin_config);
        _robot_ptr[robot_id] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>()));
}

void mgnss::plugins::RosBase::_initModule(YAML::Node config, YAML::Node plugin_config){
        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][_name];
        config["name"] = _name;
        _resetPrt(config);
        _initCallbacks(config);

        for(auto& robot: _robot_ptr){
          robot.second->get();
          robot.second->updateKinematics();
        }
}


std::string mgnss::plugins::RosBase::_loadConfig(YAML::Node& config, YAML::Node& plugin_config)
{
        std::string config_file;

        if (!_n->getParam("/mwoibn_config", config_file) && !_n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        // Read MWOIBN config file
        config = mwoibn::robot_class::Robot::getConfig(config_file);

        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

        return config_file;
}


bool mgnss::plugins::RosBase::init()
{
        YAML::Node config, plugin_config;

        std::string config_file = _loadConfig(config, plugin_config);

        std::string secondary_file = _checkConfig(plugin_config, config_file);

        _loadRobot(config_file, secondary_file, config, plugin_config);
        _logger_ptr.reset(new mwoibn::common::RosLogger(_name));

        _initModule(config, plugin_config);
        _controller_ptr->startLog(*_logger_ptr.get());

        return true;
}


void mgnss::plugins::RosBase::_initModule(YAML::Node config, YAML::Node plugin_config, mwoibn::communication_modules::Shared& share){
        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][_name];
        config["name"] = _name;

        _resetPrt(config);
        _initCallbacks(config);
        _initCallbacks(config, share);

        for(auto& robot: _robot_ptr){
          robot.second->get();
          robot.second->updateKinematics();
        }
}

bool mgnss::plugins::RosBase::init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name){

  _n = n;
  _name = name;

  YAML::Node config, plugin_config;

  std::string config_file = _loadConfig(config, plugin_config);

  std::string secondary_file = _checkConfig(plugin_config, config_file);

  std::string robot_name = _readRobot(config_file, secondary_file, config, plugin_config);

  if (share_robots.count(robot_name)){
    _robot_ptr[robot_name] = share_robots[robot_name];
    _robot_ptr[robot_name]->loadControllers(config, plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>());
  }
  else{
    //std::string robot_id = _readRobot(config_file, secondary_file, config, plugin_config);
    _robot_ptr[robot_name] = std::move(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>(), share, plugin_config["controller"].as<std::string>()));
    share_robots[robot_name] = _robot_ptr[robot_name];
  }
  _logger_ptr = logger_ptr;
  _initModule(config, plugin_config, share);
  _controller_ptr->initLog(*_logger_ptr.get());

}

void mgnss::plugins::RosBase::start(double time)
{
        _start = time;

        for(auto& robot: _robot_ptr)  _valid = robot.second->get() && _valid;


        _rate = true;

        if (_valid)
        {
          for(auto& robot: _robot_ptr) robot.second->updateKinematics();
                _controller_ptr->init();
                _initialized = true;

        }

}

void mgnss::plugins::RosBase::stop() {
        _controller_ptr->stop();
}

void mgnss::plugins::RosBase::control_loop(double time)
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
                        _controller_ptr->init();

                if(_rate && _valid)
                        _initialized = true;
        }

        _controller_ptr->update();
        _controller_ptr->send();
        _controller_ptr->log(*_logger_ptr.get(), time-_start);
        _logger_ptr->write();
        _robot_ptr.begin()->second->wait();

}

bool mgnss::plugins::RosBase::close() {
        _controller_ptr->close();
        _logger_ptr->flush();
        _logger_ptr->close();
        return true;
}
