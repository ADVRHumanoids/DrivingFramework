#ifndef __MGNSS_PLUGINS_XBOT_BASE_H
#define __MGNSS_PLUGINS_XBOT_BASE_H
#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/modules/base.h>
#include <mwoibn/common/xbot_logger.h>

//#include <custom_services/jointStateCmnd.h>

namespace mgnss
{
namespace plugins
{
class XbotBase : public XBot::XBotControlPlugin
{

public:
  virtual bool init_control_plugin(XBot::Handle::Ptr handle);

  virtual bool close();

  virtual void on_start(double time);

  virtual void on_stop(double time);

protected:
  virtual void control_loop(double time, double period);
  virtual void _setRate(double period){
    _controller_ptr->setRate(period);}
  virtual std::string _setName() = 0;
  virtual void _resetPrt(YAML::Node config) = 0;
  virtual void _initCallbacks(XBot::Handle::Ptr handle) = 0;
  std::unique_ptr<mgnss::modules::Base> _controller_ptr;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  std::unique_ptr<mwoibn::common::Logger> _logger_ptr;

//  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
  bool _initialized = false, _valid = false, _rate = false;
  std::string _name = "";
  double _start;

};
}
}
#endif // RT_MY_TEST_H
