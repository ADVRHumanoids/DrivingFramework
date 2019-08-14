#ifndef __MGNSS__ROS_CALLBACKS_WHEELS_CONTROLLER_ZMP_H
#define __MGNSS__ROS_CALLBACKS_WHEELS_CONTROLLER_ZMP_H

#include "mgnss/controllers/wheels_zmp_II.h"
#include <mgnss_utils/force.h>


namespace mgnss
{
namespace ros_callbacks

{
namespace wheels_controller_zmp
{
  bool forceLimits(mgnss_utils::force::Request& req,
                   mgnss_utils::force::Response& res, mgnss::controllers::WheelsZMPII* controller_ptr);


}
}
}
#endif // WHEELED_MOTION_H
