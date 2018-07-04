#ifndef __MGNSS__ROS_CALLBACKS_WHEELS_CONTROLLER_EVENTS_H
#define __MGNSS__ROS_CALLBACKS_WHEELS_CONTROLLER_EVENTS_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include <custom_messages/StateMsg.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_controller_events
{
bool eventsHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsControllerExtend* controller_ptr);
void supportHandler(const custom_messages::CustomCmndConstPtr& msg, mwoibn::VectorN* support, mgnss::controllers::WheelsControllerExtend* controller_ptr);
bool stateHandler(const custom_messages::StateMsgConstPtr& msg, mgnss::controllers::WheelsControllerExtend* controller_ptr);
}
}
}
#endif // WHEELED_MOTION_H
