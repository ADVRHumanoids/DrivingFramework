#include <mgnss/plugins/xbot_base.h>
//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)

bool mgnss::plugins::XbotBase::init_control_plugin(XBot::Handle::Ptr handle)
{

  // Read XBotCore config file
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());

  _name = _setName();

  // Read path to mwoibn config file
  if (!config["config_file"])
    throw std::invalid_argument(handle->getPathToConfigFile() +
        std::string("\t Please specify MWOIBN config file."));

  std::string config_file = config["config_file"].as<std::string>();

  // get plugin parameters
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

  // read a secondary file for the plugin
  std::string secondary_file = "";
  if (plugin_config["secondary_file"])
    secondary_file = plugin_config["secondary_file"].as<std::string>();

  config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

  config["robot"]["layer"] = plugin_config["layer"].as<std::string>();
  config["robot"]["mode"] = plugin_config["mode"].as<std::string>();

  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config, plugin_config["robot"].as<std::string>(), handle->getSharedMemory()));

  config = mwoibn::robot_class::Robot::readFullConfig(config, plugin_config["robot"].as<std::string>());
  config = config["modules"][_name];

  _resetPrt(config);
  _initCallbacks(handle);

  _logger_ptr.reset(new mwoibn::common::XbotLogger(_name));
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
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
    }

}

void mgnss::plugins::XbotBase::on_stop(double time) {
  _controller_ptr->stop();
}

void mgnss::plugins::XbotBase::control_loop(double time, double period)
{

    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {

      if(!_rate){
       _setRate(period); // here I may need a controller method
       _rate = true;
      }
       if(_valid){
         _valid = _robot_ptr->feedbacks.reset();
         _controller_ptr->init();
       }
       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _controller_ptr->send();
    _controller_ptr->log(*_logger_ptr.get(), time-_start);

//   std::cout <<  _robot_ptr->command.get(mwoibn::robot_class::INTERFACE::VELOCITY).transpose() << std::endl;

}

bool mgnss::plugins::XbotBase::close() {
  _controller_ptr->close();
  _logger_ptr->flush();
  _logger_ptr->close();
  return true; }

