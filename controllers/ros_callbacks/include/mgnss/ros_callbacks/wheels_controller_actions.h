#ifndef __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_ACTIONS_H
#define __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_ACTIONS_H

#include "mgnss/controllers/wheeled_motion_actions.h"
#include "mgnss/ros_callbacks/wheels_controller_extend.h"

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_controller_actions
{
bool eventsHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res, mgnss::controllers::WheeledMotionActions* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
