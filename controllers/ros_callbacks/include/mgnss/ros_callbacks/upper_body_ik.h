#ifndef __MGNSS__ROS_CALLBACKS_UPPER_BODY_IK_H
#define __MGNSS__ROS_CALLBACKS_UPPER_BODY_IK_H

#include "mgnss/controllers/upper_body_IK.h"
#include "mgnss_utils/point.h"

namespace mgnss
{
namespace ros_callbacks
{
namespace upper_body_ik
{
bool setReference(mgnss_utils::point::Request& req,
                      mgnss_utils::point::Response& res, mgnss::controllers::UpperBodyIK* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
