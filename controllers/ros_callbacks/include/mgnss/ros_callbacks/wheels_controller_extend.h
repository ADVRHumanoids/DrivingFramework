#ifndef __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_EVENT_H
#define __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_EVENT_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_controller_extend
{
bool eventsHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsControllerExtend* controller_ptr);
void supportHandler(const custom_messages::CustomCmndConstPtr& msg, mwoibn::VectorN* support, mgnss::controllers::WheelsControllerExtend* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
