#include "mwoibn/communication_modules/custom_controller.h"

bool mwoibn::communication_modules::CustomController::send()
{

  if (_position)
  {
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
                    _des_q.position);
  }
  if (_torque)
  {
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE),
                    _des_q.effort);
  }

  _command_pub.publish(_des_q);

  return true;
}
