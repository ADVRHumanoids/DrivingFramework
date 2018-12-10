#ifndef __MGNSS__ROS_CALLBACKS__WHEELS_ZMP_H
#define __MGNSS__ROS_CALLBACKS__WHEELS_ZMP_H

#include "mgnss/controllers/wheels_zmp.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include <custom_messages/StateMsg.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_zmp
{
bool eventsHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsZMP* controller_ptr);
void supportHandler(const custom_messages::CustomCmndConstPtr& msg, mgnss::controllers::WheelsZMP* controller_ptr);
bool stateHandler(const custom_messages::StateMsgConstPtr& msg, mgnss::controllers::WheelsZMP* controller_ptr);
}
}
}
#endif // WHEELED_MOTION_H
