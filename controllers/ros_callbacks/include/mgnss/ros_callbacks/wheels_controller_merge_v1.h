#ifndef __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_MERGE_V1_H
#define __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_MERGE_V1_H

#include "mgnss/controllers/wheeled_motion_merge_v1.h"
#include "mgnss/ros_callbacks/wheels_controller_extend.h"

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_controller_merge
{
bool eventsHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res, mgnss::controllers::WheeledMotionMergeV1* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
