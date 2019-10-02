#ifndef __MGNSS__ROS_CALLBACKS__WHEELS_CONTROLLER_SECOND_ORDER_H
#define __MGNSS__ROS_CALLBACKS__WHEELS_CONTROLLER_SECOND_ORDER_H

#include "mgnss/controllers/wheels_second_order.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>
#include <custom_messages/StateMsg.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace wheels_controller_second_order
{

  bool eventsHandler(custom_services::updatePDGains::Request& req,
                     custom_services::updatePDGains::Response& res, mgnss::controllers::WheelsSecondOrder* controller_ptr);
  void supportHandler(const custom_messages::CustomCmndConstPtr& msg, mgnss::controllers::WheelsSecondOrder* controller_ptr);
  bool stateHandler(const custom_messages::StateMsgConstPtr& msg, mgnss::controllers::WheelsSecondOrder* controller_ptr);
}
}
}
#endif // WHEELED_MOTION_H
