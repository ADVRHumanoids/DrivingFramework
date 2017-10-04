#include "mwoibn/communication_modules/ros_controller.h"

bool mwoibn::communication_modules::RosController::send()
{

  if (_position)
  {
//    pub_qq.resize(_dofs, 0);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), _des_q.position);
    //des_q.position = pub_qq;
  }
  if (_velocity)
  {
   // pub_qq.resize(_dofs, 0);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), _des_q.velocity);
    //des_q.velocity = pub_qq;
  }

  if (_torque)
  {
    //pub_qq.resize(_dofs, 0);
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _des_q.effort);
    //des_q.effort = pub_qq;
  }

  // des_q.header.stamp = ros::Time::now();
  _command_pub.publish(_des_q);

  return true;
}
