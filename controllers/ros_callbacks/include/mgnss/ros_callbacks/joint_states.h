#ifndef __MGNSS__ROS_CALLBACKS_JOINT_STATES_H
#define __MGNSS__ROS_CALLBACKS_JOINT_STATES_H

#include <mgnss/controllers/joint_states.h>
#include <custom_services/jointStateCmnd.h>

namespace mgnss
{
namespace ros_callbacks
{
namespace joint_states
{
  bool referenceHandler(custom_services::jointStateCmnd::Request& req,
                               custom_services::jointStateCmnd::Response& res, mgnss::controllers::JointStates* controller_ptr);

}
}
}
#endif // WHEELED_MOTION_H
