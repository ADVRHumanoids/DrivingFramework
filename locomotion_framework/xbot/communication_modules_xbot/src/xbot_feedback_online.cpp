#include "mwoibn/communication_modules/xbot_feedback_online.h"

bool mwoibn::communication_modules::XBotFeedbackOnline::get()
{

  if (_position){
  //   _robot.getJointPosition(_pub);
    _robot.getMotorPosition(_pub);
    _command.set(_pub, _map.reversed(), robot_class::INTERFACE::POSITION);

  }
  if (_velocity){
    _robot.getJointVelocity(_pub);
    _command.set(_pub, _map.reversed(),
                 robot_class::INTERFACE::VELOCITY);
  }
  if (_torque){
    _robot.getJointEffort(_pub);
    _command.set(_pub, _map.reversed(),
                 robot_class::INTERFACE::TORQUE);
  }

  return true;
}
