#include <mgnss/xbot_plugins/joint_states.h>

REGISTER_XBOT_PLUGIN(JointStates, mgnss::xbot_plugins::JointStates)

bool mgnss::xbot_plugins::JointStates::init_control_plugin(XBot::Handle::Ptr handle)
{

  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());
  std::string config_file = config["config_file"].as<std::string>();
  config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["joint_states"];

  std::string secondary_file = "";
  if (config["secondary_file"])
    secondary_file = config["secondary_file"].as<std::string>();

  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config_file, config["robot"].as<std::string>(), secondary_file, handle->getSharedMemory()));

  _controller_ptr.reset(new mgnss::controllers::JointStates(*_robot_ptr));

  _srv_rt = handle->getRosHandle()->advertiseService<custom_services::jointStateCmnd::Request,
                           custom_services::jointStateCmnd::Response>(
            "trajectory", boost::bind(&mgnss::xbot_plugins::JointStates::referenceHandler, _1, _2, _controller_ptr.get()));
  _robot_ptr->get();
  _robot_ptr->updateKinematics();

/*
  _robot_ptr->update();
*/
  return true;
}

void mgnss::xbot_plugins::JointStates::on_start(double time)
{
    _valid = _robot_ptr->get();

    if (_valid)
    {
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
    }

}

void mgnss::xbot_plugins::JointStates::on_stop(double time) {}

void mgnss::xbot_plugins::JointStates::control_loop(double time, double period)
{

    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
       if(_valid)
      _controller_ptr->init();
       if(!_rate){
           _robot_ptr->setRate(period);
        _rate = true;
       }
       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _controller_ptr->send();

//   std::cout <<  _robot_ptr->command.get(mwoibn::robot_class::INTERFACE::VELOCITY).transpose() << std::endl;



}

bool mgnss::xbot_plugins::JointStates::close() { return true; }

