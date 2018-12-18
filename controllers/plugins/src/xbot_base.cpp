#include "mgnss/plugins/xbot_base.h"
//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)

bool mgnss::plugins::XbotBase::init_control_plugin(XBot::Handle::Ptr handle)
{
        _n = handle->getRosHandle();
        // Read XBotCore config file
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());


        // Read path to mwoibn config file
        if (!config["config_file"])
                throw std::invalid_argument(handle->getPathToConfigFile() +
                                            std::string("\t Please specify MWOIBN config file."));

        std::string config_file = config["config_file"].as<std::string>();

        // get plugin parameters
        YAML::Node plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

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
        // read a secondary file for the plugin
        std::string secondary_file = "";
        if (plugin_config["secondary_file"])
              secondary_file = mwoibn::robot_class::Robot::readPath(plugin_config["secondary_file"]);

        config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

        config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
        config["robot"]["mode"] = plugin_config["mode"].as<std::string>();



        _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), plugin_config["controller"].as<std::string>(), handle->getSharedMemory()));

        config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
        config = config["modules"][_name];

        _resetPrt(config);
        _initCallbacks(config);

        _logger_ptr.reset(new mwoibn::common::XbotLogger(_name));
        _logger_ptr->add("update", 0.0);
        _controller_ptr->startLog(*_logger_ptr.get());

        _robot_ptr->get();
        _robot_ptr->updateKinematics();

        return true;
}

void mgnss::plugins::XbotBase::on_start(double time)
{
        _start = time;

        _valid = _robot_ptr->get() && _robot_ptr->feedbacks.reset();

        if (_valid)
        {
                _setRate(_robot_ptr->rate());
                _robot_ptr->updateKinematics();
                _controller_ptr->init();
                _initialized = true;
        }

}

void mgnss::plugins::XbotBase::on_stop(double time) {
        _controller_ptr->stop();
}

void mgnss::plugins::XbotBase::control_loop(double time)
{
        //_begin = std::chrono::high_resolution_clock::now();

        _valid = _robot_ptr->get();

        if (!_valid)
                return;

        _robot_ptr->updateKinematics();

        if (!_initialized)
        {
                _setRate(_robot_ptr->rate());
                _valid = _robot_ptr->feedbacks.reset();
                if(_valid) {
                        _controller_ptr->init();
                        _initialized = true;
                }
        }

        _controller_ptr->update();
        _controller_ptr->send();

        //_end = std::chrono::high_resolution_clock::now();

        //_logger_ptr->add("update", std::chrono::duration_cast<std::chrono::microseconds>((_end-_begin)).count());
        _controller_ptr->log(*_logger_ptr.get(), time-_start);
        _logger_ptr->write();


}

bool mgnss::plugins::XbotBase::close() {
        _controller_ptr->close();
        _logger_ptr->flush();
        _logger_ptr->close();
        return true;
}
