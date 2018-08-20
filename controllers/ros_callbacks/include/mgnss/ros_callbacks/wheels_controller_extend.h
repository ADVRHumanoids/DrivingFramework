#ifndef __MGNSS__ROS_CALLBACKS__WHEELED_MOTION_EXTEND_H
#define __MGNSS__ROS_CALLBACKS__WHEELED_MOTION_EXTEND_H

#include "mgnss/controllers/wheels_controller_extend.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include <custom_messages/StateMsg.h>

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
