#include "mgnss/plugins/ros_base.h"
#include <mwoibn/loaders/robot.h>
#include <config.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)

mgnss::plugins::RosBase::RosBase(int argc, char** argv)
{
        //_init(argc, argv);
}
void mgnss::plugins::RosBase::_init(int argc, char** argv)
{
        _name = _setName();

        ros::init(argc, argv, _name); // initalize node needed for the service // I can put it in the NRT default plugin (?)
        _n.reset(new ros::NodeHandle());

        // ros::NodeHandle n;
}

std::string mgnss::plugins::RosBase::_checkConfig(YAML::Node plugin_config, std::string config_file)
{
        if (!plugin_config["modules"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find modules configuration."));
        if (!plugin_config["modules"][_name])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find ") + _name + std::string(" module configuration."));

        plugin_config = plugin_config["modules"][_name];

        if (!plugin_config["robot"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find robot configuration in module parameters."));
        if (!plugin_config["layer"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Please specify robot layer."));
        if (!plugin_config["mode"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Please specify controller mode."));


        std::string secondary_file = "";
        if (plugin_config["secondary_file"])
                secondary_file = plugin_config["secondary_file"].as<std::string>();

        std::cout << "secondary file " << secondary_file << std::endl;

        return secondary_file;

}

void mgnss::plugins::RosBase::_loadRobot(std::string config_file, std::string secondary_file, YAML::Node config, YAML::Node plugin_config){
        config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

        config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
        config["robot"]["mode"] = plugin_config["mode"].as<std::string>();

        _robot_ptr.reset(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>()).release());
}

void mgnss::plugins::RosBase::_initModule(YAML::Node config, YAML::Node plugin_config){
        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][_name];

        _resetPrt(config);
        _initCallbacks();

        _logger_ptr.reset(new mwoibn::common::RosLogger(_name));
        _controller_ptr->startLog(*_logger_ptr.get());

        _robot_ptr->get();
        _robot_ptr->updateKinematics();
}


std::string mgnss::plugins::RosBase::_loadConfig(YAML::Node& config, YAML::Node& plugin_config)
{
        std::string config_file;
        
        if (!_n->getParam("/mwoibn_config", config_file) && !_n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        //std::string config_file = std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/configs/mwoibn/configs/mwoibn_2_5.yaml"; // for now, later take it as a parameter

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

        _initModule(config, plugin_config);

        return true;
}

void mgnss::plugins::RosBase::start(double time)
{
        _start = time;

        _valid = _robot_ptr->get();
        _rate = true;

        if (_valid)
        {
                _robot_ptr->updateKinematics();
                _controller_ptr->init();
                _initialized = true;

        }

}

void mgnss::plugins::RosBase::stop() {
        _controller_ptr->stop();
}

void mgnss::plugins::RosBase::control_loop(double time)
{

        _valid = _robot_ptr->get();

        if (!_valid)
                return;

        _robot_ptr->updateKinematics();

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

        _robot_ptr->wait();

}

bool mgnss::plugins::RosBase::close() {
        _controller_ptr->close();
        _logger_ptr->flush();
        _logger_ptr->close();
        return true;
}
