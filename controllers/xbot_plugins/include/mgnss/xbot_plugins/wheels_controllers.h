#ifndef __MGNSS_XBOT_PLUGINS_WHEELS_CONTROLLERS_H
#define __MGNSS_XBOT_PLUGINS_WHEELS_CONTROLLERS_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/controllers/wheeled_motion_world.h"

#include "mgnss/ros_callbacks/wheels_controller_extend.h"

#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include "mgnss/plugins/xbot_base.h"

namespace mgnss
{
namespace xbot_plugins
{
class WheelsControllerExtend : public plugins::XbotBase
{

public:

  WheelsControllerExtend(int argc, char** argv) : mgnss::plugins::XbotBase(argc, argv, "wheeled_motion"){
  }

  WheelsControllerExtend() : mgnss::plugins::XbotBase("wheeled_motion"){}

  virtual ~WheelsControllerExtend(){}


virtual bool init_control_plugin(XBot::Handle::Ptr handle);

virtual void on_start(double time);

mgnss::controllers::WheelsControllerExtend& get(){
        return static_cast<mgnss::controllers::WheelsControllerExtend&>(*_controller_ptr);
}

protected:
virtual void control_loop(double time);

//  virtual void _resetPrt(YAML::Node config)
//  {
//    _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent(*_robot_ptr, config));
//  }

virtual void _initCallbacks(YAML::Node config)
{
        _srv_rt = _n->advertiseService<custom_services::updatePDGains::Request,
                                                           custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::eventsHandler,
                                                                                                                                   _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));

        //_sub_rt = handle->getRosHandle()->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, &mgnss::xbot_plugins::WheelsV2::supportHandler, this);
        _sub_rt = _n->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_extend::supportHandler,
                                                                                                                  _1,  static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));
}


XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
XBot::RosUtils::SubscriberWrapper::Ptr _sub_rt;

};

}
}
#endif // RT_MY_TEST_H
