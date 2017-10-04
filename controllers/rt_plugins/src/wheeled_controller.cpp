#include <rt_plugins/wheeled_controller.h>

REGISTER_XBOT_PLUGIN(WheeledController, mwoibn::WheeledController)

namespace mwoibn
{

bool WheeledController::init_control_plugin(
    std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory,
    XBot::RobotInterface::Ptr robot)
{
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
      path_to_config_file, "higher_scheme", shared_memory));

  _controller_ptr.reset(new mwoibn::WheeledMotion(*_robot_ptr));

  _sub_references.init("wheels_reference");

  _references << 0.4, 0.22, 0.4, -0.22, -0.4, 0.22, -0.4, -0.22, -0.04, 0, 0.40,
      0;
  return true;
}

void WheeledController::on_start(double time)
{
  _readReference();

  _initialized = _robot_ptr->get();

  if(_initialized) {
    _robot_ptr->updateKinematics();
    _controller_ptr->init();
  }

}

void WheeledController::on_stop(double time) {}
void WheeledController::_readReference() {

  Eigen::Matrix<double, 13, 1> references;

  _sub_references.read(references);

  if(references[12] == mwoibn::IS_VALID) _references = references.head(12);


}

void WheeledController::control_loop(double time, double period)
{
  _valid = _robot_ptr->get();

  if(!_valid) return;

  _robot_ptr->updateKinematics();

  if (!_initialized)
  {
    _controller_ptr->init();
    _initialized = true;
  }

  _readReference();

  _controller_ptr->update(_references.head(8), _references.segment<3>(8),
                              _references[11]);

  _robot_ptr->send();
  //_robot.wait();
}

bool WheeledController::close() { return true; }
}
