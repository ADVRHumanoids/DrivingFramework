#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_3_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_3_H

#include "mgnss/controllers/wheeled_motion_event_v3.h"
#include "mgnss/controllers/wheeled_motion_event.h"
#include "mgnss/xbot_plugins/wheels_controllers.h"
#include "mgnss/ros_callbacks/wheels_controller_events.h"

#include <custom_messages/StateMsg.h>

namespace mgnss
{
namespace xbot_plugins
{

class WheeledMotionEvent3 : public WheelsControllerExtend {
//  public:
//    WheeledMotionEvent3(int argc, char** argv): WheelsControllerExtend(argc, argv){}
//    ~WheeledMotionEvent3(){}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent3(*_robot_ptr, config));
}


virtual void _initCallbacks(XBot::Handle::Ptr handle){
        _srv_rt = handle->getRosHandle()->advertiseService<custom_services::updatePDGains::Request,
                                       custom_services::updatePDGains::Response>("wheels_command", boost::bind(&mgnss::ros_callbacks::wheels_controller_events::eventsHandler,
                                                                                                               _1, _2, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));

        _sub_rt = handle->getRosHandle()->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::supportHandler,_1, &_support, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));
        
		_state_rt = handle->getRosHandle()->subscribe<custom_messages::StateMsg>("wheels_state", 1, boost::bind(&mgnss::ros_callbacks::wheels_controller_events::stateHandler,_1, static_cast<mgnss::controllers::WheelsControllerExtend*>(_controller_ptr.get())));

}
//ros::Subscriber _state_rt;
XBot::RosUtils::SubscriberWrapper::Ptr _state_rt;
};

}
}
#endif // RT_MY_TEST_H
