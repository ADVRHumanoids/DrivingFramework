#ifndef __MGNSS_XBOT_PLUGINS_ODOMETRY_H
#define __MGNSS_XBOT_PLUGINS_ODOMETRY_H

//#include <XCM/XBotControlPlugin.h>
//#include <mwoibn/robot_class/robot_xbot_rt.h>
#include "mgnss/state_estimation/odometry.h"
#include <fstream>
#include "mgnss/plugins/xbot_base.h"

namespace mgnss
{
namespace xbot_plugins
{
class Odometry : public mgnss::plugins::XbotBase
{

public:
  Odometry(int argc, char** argv) : mgnss::plugins::XbotBase(argc, argv, "odometry"){
  }

  Odometry() : mgnss::plugins::XbotBase("odometry"){}

  virtual ~Odometry(){}



protected:
virtual void _resetPrt(YAML::Node config)
{
        _controller_ptr.reset(new mgnss::state_estimation::Odometry(*_robot_ptr, config));
}
virtual void _initCallbacks(YAML::Node config)
{
}

};
}
}

#endif // RT_MY_TEST_H
