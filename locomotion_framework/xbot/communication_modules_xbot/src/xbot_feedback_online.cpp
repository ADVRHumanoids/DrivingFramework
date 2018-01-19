#include "mwoibn/communication_modules/xbot_feedback_online.h"

bool mwoibn::communication_modules::XBotFeedbackOnline::get()
{

  if (_position){
//     _robot.getJointPosition(_pub);
    _robot.getMotorPosition(_pub);
//    std::cout << "feedback\t" << _pub.transpose() << std::endl;
    _command.set(_pub, _map.reversed(), robot_class::INTERFACE::POSITION);
//    std::cout << "position feedback " << _command.get(robot_class::INTERFACE::POSITION).transpose() << std::endl;

  }
  if (_velocity){
    _robot.getJointVelocity(_pub);
    _command.set(_pub, _map.reversed(),
                 robot_class::INTERFACE::VELOCITY);
    std::cout << "velocity feedback " << _command.get(robot_class::INTERFACE::VELOCITY).transpose() << std::endl;
  }
  if (_torque){
    _robot.getJointEffort(_pub);
    _command.set(_pub, _map.reversed(),
                 robot_class::INTERFACE::TORQUE);
  }

  return true;
}
