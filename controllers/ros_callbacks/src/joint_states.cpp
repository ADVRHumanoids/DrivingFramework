#include "mgnss/ros_callbacks/joint_states.h"

bool mgnss::ros_callbacks::joint_states::referenceHandler(custom_services::jointStateCmnd::Request& req,
                             custom_services::jointStateCmnd::Response& res, mgnss::controllers::JointStates* controller_ptr)
{

  if (controller_ptr->setFullPosition(req.position))
  {
    res.message = "Found requested position " + req.position;
    // res.message = "Position " + req.position + " has not been defined in
    // the robot";
  }
  else if (controller_ptr->setVelocity(req.position, req.velocity))
    res.message = "Set requested velocity for " + req.position;
  else if (controller_ptr->setPosition(req.position, req.velocity))
    res.message = "Set requested position for " + req.position;
  else
    res.message = "Unknown command " + req.position + ".";

  if (req.pos_step)
    controller_ptr->step(req.pos_step);

  res.success = true;

  return true;
}
