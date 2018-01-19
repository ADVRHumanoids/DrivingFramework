#include <mgnss/xbot_plugins/wheeled_controller_v2.h>

REGISTER_XBOT_PLUGIN(WheelsV2, mgnss::xbot_plugins::WheelsV2)

bool mgnss::xbot_plugins::WheelsV2::init_control_plugin(
    XBot::Handle::Ptr handle)
{
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
      handle->getRobotInterface(), handle->getPathToConfigFile(), "robot", handle->getSharedMemory()));

  _controller_ptr.reset(new mwoibn::WheeledMotionEvent(*_robot_ptr));

  _srv_rt = handle->getRosHandle()->advertiseService("wheels_command", &mgnss::xbot_plugins::WheelsV2::evenstHandler, this);

  return true;
}

void mgnss::xbot_plugins::WheelsV2::on_start(double time)
{
    _valid = _robot_ptr->get();

    if (_valid)
    {
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
      _support = _controller_ptr->getSupportReference();
    }
}

void mgnss::xbot_plugins::WheelsV2::on_stop(double time) {

    _controller_ptr->stop();
}


void mgnss::xbot_plugins::WheelsV2::control_loop(double time,
                                                          double period)
{
    _valid = _robot_ptr->get();

    if (!_valid)
      return;


    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
       if(_valid){
      _controller_ptr->init();
      _support = _controller_ptr->getSupportReference();
       }
       if(!_rate){
           _robot_ptr->setRate(period);
           _controller_ptr->setRate();
           _rate = true;
       }
       if(_rate && _valid){
        _initialized = true;
       }

    }


//    std::cout << "base\t" << _robot_ptr->state.get().head(6).transpose() << std::endl;
    _controller_ptr->update(_support);
    _robot_ptr->send();

}

bool mgnss::xbot_plugins::WheelsV2::close() { return true; }
