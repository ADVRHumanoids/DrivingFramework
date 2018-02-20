#include <mgnss/xbot_plugins/odometry.h>

REGISTER_XBOT_PLUGIN(Odometry, mgnss::xbot_plugins::Odometry)

bool mgnss::xbot_plugins::Odometry::init_control_plugin(XBot::Handle::Ptr handle)
{
  /* Save robot to a private member. */
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());
  std::string config_file = config["config_file"].as<std::string>();
  config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["odometry"];

  std::string secondary_file = "";
  if (config["secondary_file"])
    secondary_file = config["secondary_file"].as<std::string>();

  /* Save robot to a private member. */
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config_file, config["robot"].as<std::string>(), secondary_file, handle->getSharedMemory()));

//  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), handle->getPathToConfigFile(), "robot2", handle->getSharedMemory()));

  _controller_ptr.reset(new mgnss::odometry::Odometry(*_robot_ptr, {"wheel_1", "wheel_2", "wheel_3", "wheel_4"}, 0.078));

  _robot_ptr->update();

  return true;
}

void mgnss::xbot_plugins::Odometry::on_start(double time)
{
    _valid = _robot_ptr->get();

    if (_valid)
    {
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
    }
}

void mgnss::xbot_plugins::Odometry::on_stop(double time) {}

void mgnss::xbot_plugins::Odometry::control_loop(double time, double period)
{
    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
      if(!_rate){
          _robot_ptr->setRate(period);
          _controller_ptr->setRate();
       _rate = true;
      }
       if(_valid)
      _controller_ptr->init();

       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _robot_ptr->send();

}

bool mgnss::xbot_plugins::Odometry::close() { return true; }

