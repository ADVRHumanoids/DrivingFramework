#include <mgnss/rt_plugins/combined_controller.h>

REGISTER_XBOT_PLUGIN(CombinedController, mgnss::rt_plugins::CombinedController)


bool mgnss::rt_plugins::CombinedController::init_control_plugin(
    std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory,
    XBot::RobotInterface::Ptr robot)
{
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
      path_to_config_file, "robot1", shared_memory));

  _wheels_controller_ptr.reset(new mwoibn::WheeledMotion(*_robot_ptr));

  _centralized_controller_ptr.reset(new mgnss::controllers::OnlineCentralizedController(*_robot_ptr));

  _sub_references.init("wheels_reference");

  _references << 0.4, 0.22, 0.4, -0.22, -0.4, 0.22, -0.4, -0.22, -0.04, 0, 0.40,
      0;
  return true;
}

void mgnss::rt_plugins::CombinedController::on_start(double time)
{
  _readReference();

  _initialized = _robot_ptr->get();

  if(_initialized) {
    _robot_ptr->updateKinematics();
    _wheels_controller_ptr->init();
  }

}

void mgnss::rt_plugins::CombinedController::on_stop(double time) {}

void mgnss::rt_plugins::CombinedController::_readReference() {

  Eigen::Matrix<double, 13, 1> references;

  _sub_references.read(references);

  if(references[12] == mwoibn::IS_VALID) _references = references.head(12);

//  std::cout << _references << std::endl;
}

void mgnss::rt_plugins::CombinedController::control_loop(double time, double period)
{

  if(!_robot_ptr->get()) return;

  _robot_ptr->updateKinematics();

  if (!_initialized)
  {
    _wheels_controller_ptr->init();
    _initialized = true;
  }

  _readReference();

  _wheels_controller_ptr->update(_references.head(8), _references.segment<3>(8),
                              _references[11]);

  _centralized_controller_ptr->update();

  _robot_ptr->send();
}

bool mgnss::rt_plugins::CombinedController::close() { return true; }
