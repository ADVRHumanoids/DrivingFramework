#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_CONTROLLER_V2_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_CONTROLLER_V2_H

//#include <XCM/XBotControlPlugin.h>

//#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/controllers/wheeled_motion_event.h>
#include <mgnss/ros_callbacks/wheeled_motion_event.h>

#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include <mgnss/plugins/xbot_base.h>

namespace mgnss
{
namespace xbot_plugins
{
class WheelsV2 : public plugins::XbotBase
{

public:
  virtual bool init_control_plugin(XBot::Handle::Ptr handle);

  virtual void on_start(double time);
  mgnss::controllers::WheeledMotionEvent& get(){return static_cast<mgnss::controllers::WheeledMotionEvent&>(*_controller_ptr);}

protected:
  virtual void control_loop(double time, double period);

  virtual void _resetPrt(YAML::Node config)
  {
    _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent(*_robot_ptr, config));
  }

  virtual void _initCallbacks(XBot::Handle::Ptr handle)
  {
    _srv_rt = handle->getRosHandle()->advertiseService<custom_services::updatePDGains::Request,
        custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheeled_motion_event::evenstHandler,
                                                                                     _1, _2, static_cast<mgnss::controllers::WheeledMotionEvent*>(_controller_ptr.get())));

    //_sub_rt = handle->getRosHandle()->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, &mgnss::xbot_plugins::WheelsV2::supportHandler, this);
    _sub_rt = handle->getRosHandle()->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheeled_motion_event::supportHandler,
                                                                                                              _1, &_support,  static_cast<mgnss::controllers::WheeledMotionEvent*>(_controller_ptr.get())));
  }

  virtual std::string _setName(){return "wheeled_motion";}

private:
//  std::ostringstream oss;
//  std::ofstream file;
//  Eigen::IOFormat fmt;

//  std::time_t t;
//  std::tm tm;
//  double start,now;
//  mwoibn::VectorN _print;


//  std::unique_ptr<mgnss::controllers::WheeledMotionEvent> _controller_ptr;

  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
  XBot::RosUtils::SubscriberWrapper::Ptr _sub_rt;
  mwoibn::VectorN _support;
  //XBot::MatLogger::Ptr _logger;
  mwoibn::Vector3 _log_point;
  double start;

};
}
}
#endif // RT_MY_TEST_H
