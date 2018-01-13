#include <mgnss/xbot_plugins/online_centralized_controller_plugin.h>

REGISTER_XBOT_PLUGIN(OnlineCentralizedControllerPlugin,
                     mgnss::xbot_plugins::OnlineCentralizedControllerPlugin)

bool mgnss::xbot_plugins::OnlineCentralizedControllerPlugin::init_control_plugin(
    std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory,
    XBot::RobotInterface::Ptr robot)
{

  /* Save robot to a private member. */
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
      path_to_config_file, "robot1", shared_memory));
  _robot_ref_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
      path_to_config_file, "robot2", shared_memory));

  _controller_ptr.reset(new mgnss::controllers::OnlineCentralizedController(
      *_robot_ptr));

  return true;
}

void mgnss::xbot_plugins::OnlineCentralizedControllerPlugin::on_start(double time)
{
  //  _start_time = time;
}

void mgnss::xbot_plugins::OnlineCentralizedControllerPlugin::on_stop(double time)
{
}

void mgnss::xbot_plugins::OnlineCentralizedControllerPlugin::control_loop(
    double time, double period)
{
  if (!_robot_ptr->get())
    return;

  if (_robot_ref_ptr->get())
  {
    _robot_ptr->command.set(
        _robot_ref_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION),
        mwoibn::robot_class::INTERFACE::POSITION);
  }

  _robot_ptr->updateKinematics();

  _controller_ptr->update();
  _robot_ptr->send();
}

bool mgnss::xbot_plugins::OnlineCentralizedControllerPlugin::close()
{
  return true;
}
