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

bool mgnss::plugins::RosBase::init()
{

        std::string config_file = std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/configs/mwoibn/mwoibn_v2_5.yaml"; // for now, later take it as a parameter

        // Read MWOIBN config file
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        YAML::Node plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);


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

        config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

        config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
        config["robot"]["mode"] = plugin_config["mode"].as<std::string>();

        _robot_ptr.reset(mwoibn::loaders::Robot::create(config,  plugin_config["robot"].as<std::string>()).release());

        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][_name];

        _resetPrt(config);
        _initCallbacks();

        _logger_ptr.reset(new mwoibn::common::RosLogger(_name));
        _controller_ptr->startLog(*_logger_ptr.get());

        _robot_ptr->get();
        _robot_ptr->updateKinematics();

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
