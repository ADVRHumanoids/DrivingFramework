#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_SELF_COLLISION_CONTROLLER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_SELF_COLLISION_CONTROLLER_H

#include "mwoibn/collision_model/robot_collision.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
//#include <rbdl/rbdl.h>
#include "mwoibn/hierarchical_control/hierarchical_control.h"

namespace mwoibn
{

namespace hierarchical_control
{
namespace tasks
{

//! Provides the implementation of a one dimensional self-collision avoidance
//task
class SelfCollision : public BasicTask
{

public:
/**
 * @param robot pointer to the robot collision model
 * @param safety_limit minimum distance between the pair the self-collision
 * avoidance should turn on. All paris work with the same safety limit
 */
SelfCollision(mwoibn::collision_model::RobotCollision& robot,
              double safety_limit)
        : BasicTask(), _robot(robot)
{
        for (int i = 0; i < _robot.getPairsNumber(); i++)
                _safety_limit.push_back(safety_limit);

        _init(1, _robot.getRobotDofs());
}

/**
 * @param robot pointer to the robot collision model
 * @param safety_limit minimum distance between the pair the self-collision
 **avoidance should turn on.<BR>
 * - safety_limit.size() == 1: All paris work with the same safety limit<BR>
 * - safety_limit.size() == number of collision pairs: Each pari can have
 **diffrent safety limit<BR>
 * - otherwise throw invalid argument error
 *
 */
SelfCollision(mwoibn::collision_model::RobotCollision& robot,
              std::vector<double> safety_limit)
        : BasicTask(), _robot(robot)
{
        if (safety_limit.size() == _robot.getPairsNumber())
                _safety_limit = safety_limit;
        else if (safety_limit.size() == 1)
        {
                for (int i = 0; i < _robot.getPairsNumber(); i++)
                        _safety_limit.push_back(safety_limit[0]);
        }
        else
                throw(std::invalid_argument(
                              "Couldn't read safety limit. Wrong vector size recieved."));

        _init(1, _robot.getRobotDofs());
}
virtual ~SelfCollision() {
}
//! updates task error based on the current state of the robot
virtual void updateError();
//! updates task Jacobian based on the current state of the robot
virtual void updateJacobian();

protected:
/*		bool _init(){
                        _error = mwoibn::VectorN::Zero(1);
                        _jacobian =
   mwoibn::Matrix::Zero(1,_robot.getRobotDofs());
                        _last_error =
   mwoibn::VectorN::Zero(1);
                        _last_jacobian =
   mwoibn::Matrix::Zero(1,_robot.getRobotDofs());
        } */

//! pointer to the robot collision model
mwoibn::collision_model::RobotCollision& _robot;
//! minimum distance between the pair the self-collision avoidance should turn
//on. All paris work with the same safety limit
std::vector<double> _safety_limit;

double dof;
};
}
} // namespace package
} // namespace library
#endif
