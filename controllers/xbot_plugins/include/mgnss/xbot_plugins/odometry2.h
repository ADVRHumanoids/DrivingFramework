#ifndef __MGNSS_XBOT_PLUGINS_ODOMETRY2_H
#define __MGNSS_XBOT_PLUGINS_ODOMETRY2_H

//#include <XCM/XBotControlPlugin.h>
//#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/state_estimation/odometry_v2.h>
#include <fstream>
#include <mgnss/plugins/xbot_base.h>

namespace mgnss
{
namespace xbot_plugins
{
class Odometry2 : public mgnss::plugins::XbotBase
{
protected:
  virtual void _resetPrt(YAML::Node config)
  {
    _controller_ptr.reset(new mgnss::state_estimation::OdometryV2(*_robot_ptr, config));
  }
  virtual void _initCallbacks(XBot::Handle::Ptr handle)
  {
  }
  virtual std::string _setName(){return "odometry";}

};
}
}
#endif // RT_MY_TEST_H
