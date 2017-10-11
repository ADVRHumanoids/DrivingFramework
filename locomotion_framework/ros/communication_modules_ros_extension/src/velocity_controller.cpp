#include "mwoibn/communication_modules/velocity_controller.h"

bool mwoibn::communication_modules::VelocityController::send()
{

  if (_velocity)
  {
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
                    _des_q.velocity);
  }


  _command_pub.publish(_des_q);

  return true;
}
