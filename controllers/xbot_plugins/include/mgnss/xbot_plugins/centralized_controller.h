#ifndef __MGNSS_XBOT_PLUGINS_CENTRALIZED_CONTROLLER_H
#define __MGNSS_XBOT_PLUGINS_CENTRALIZED_CONTROLLER_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mwoibn/gravity_compensation/simple_qr_gravity_compensation.h>
#include <mwoibn/motor_side_reference/sea_reference.h>
#include <mwoibn/dynamic_models/qr_decomposition.h>

namespace mgnss
{

namespace xbot_plugins
{

class CentralizedController : public XBot::XBotControlPlugin
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
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ref_ptr;

  std::unique_ptr<mwoibn::dynamic_models::QrDecomposition>
      _dynamic_model_ptr; // online set up
  std::unique_ptr<mwoibn::gravity_compensation::SimpleQRGravityCompensation>
      _gravity_compensation_ptr;
  std::unique_ptr<mwoibn::motor_side_reference::SeaReference>
      _actuation_model_ptr;

  double _start_time;
  bool _motor_side = false;
  bool _valid = false;
};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
