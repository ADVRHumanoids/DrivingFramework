#include <mgnss/plugins/xbot_base.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)

bool mgnss::plugins::XbotBase::init_control_plugin(XBot::Handle::Ptr handle)
{


  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());

  _name = _setName();
  std::string config_file = config["config_file"].as<std::string>();
  config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][_name];

  std::string secondary_file = "";
  if (config["secondary_file"])
    secondary_file = config["secondary_file"].as<std::string>();

  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config_file, config["robot"].as<std::string>(), secondary_file, handle->getSharedMemory()));

  _resetPrt(config_file);
  _initCallbacks(handle);

  _robot_ptr->get();
  _robot_ptr->updateKinematics();

  return true;
}

void mgnss::plugins::XbotBase::on_start(double time)
{
    start = time;

    _valid = _robot_ptr->get();

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
       if(_valid)
      _controller_ptr->init();

       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _controller_ptr->send();

//   std::cout <<  _robot_ptr->command.get(mwoibn::robot_class::INTERFACE::VELOCITY).transpose() << std::endl;

}

bool mgnss::plugins::XbotBase::close() {
  _controller_ptr->close();
  return true; }

