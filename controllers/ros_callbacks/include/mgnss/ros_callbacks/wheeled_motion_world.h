#ifndef __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_WORLD_H
#define __MGNSS__ROS_CALLBACKS_WHEELED_MOTION_WORLD_H

#include <mgnss/controllers/wheeled_motion_world.h>
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace wheeled_motion_world
{
  bool evenstHandler(custom_services::updatePDGains::Request& req,
                               custom_services::updatePDGains::Response& res, mgnss::controllers::WheeledMotionWorld* controller_ptr);
  void supportHandler(const custom_messages::CustomCmndConstPtr& msg, mwoibn::VectorN* support, mgnss::controllers::WheeledMotionWorld* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
