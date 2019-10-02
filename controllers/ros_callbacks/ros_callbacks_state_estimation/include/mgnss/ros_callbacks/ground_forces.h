#ifndef __MGNSS__ROS_CALLBACKS_GROUND_FORCES_H
#define __MGNSS__ROS_CALLBACKS_GROUND_FORCES_H

#include "mgnss/state_estimation/ground_forces.h"
#include "geometry_msgs/WrenchStamped.h"

namespace mgnss
{
namespace ros_callbacks
{
namespace ground_forces
{
/*  void wrench(const geometry_msgs::WrenchStampedConstPtr& msg, mgnss::state_estimation::GroundForces* estimator_ptr)
  {
        mwoibn::Vector3 force, torque;

        force[0] = msg->wrench.force.x;
        force[1] = msg->wrench.force.y;
        force[2] = msg->wrench.force.z;

        torque[0] = msg->wrench.torque.x;
        torque[1] = msg->wrench.torque.y;
        torque[2] = msg->wrench.torque.z;

        estimator_ptr->setBase(force, torque);
  }*/
}
}
}
#endif // WHEELED_MOTION_H
