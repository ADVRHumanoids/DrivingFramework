#ifndef PROGRAM_RT_WHEELED_CONTROLLER_H
#define PROGRAM_RT_WHEELED_CONTROLLER_H

#include <XCM/XBotControlPlugin.h>

#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/controllers/wheeled_motion.h>

namespace mgnss
{
namespace xbot_plugins
{
class WheeledController : public XBot::XBotControlPlugin
{

public:
  virtual bool init_control_plugin(std::string path_to_config_file,
                                   XBot::SharedMemory::Ptr shared_memory,
                                   XBot::RobotInterface::Ptr robot);

  virtual bool close();

  virtual void on_start(double time);

  virtual void on_stop(double time);

protected:
  virtual void control_loop(double time, double period);

private:
  void _readReference();
  bool _initialized = false, _valid = false;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  std::unique_ptr<mwoibn::WheeledMotion> _controller_ptr;

  Eigen::Matrix<double, 12, 1> _references;
  XBot::SubscriberRT<Eigen::Matrix<double, 13, 1>> _sub_references;
};
}
}
#endif // RT_MY_TEST_H
