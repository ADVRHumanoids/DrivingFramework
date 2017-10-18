#include "mwoibn/communication_modules/xbot_lower_level.h"

bool mwoibn::communication_modules::XBotLowerLevel::send()
{
//  std::cout << "send()\n";
    if (_position)
    {
//        std::cout << "position" << std::endl;
//        std::cout << _command.get(mwoibn::robot_class::INTERFACE::POSITION) << std::endl;
        _robot.getPositionReference(pub);

      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION),
                      pub);
      _robot.setPositionReference(pub);
    }
    if (_velocity)
    {
//      std::cout << "velocity\n";

      //pub.setZero();
      _robot.getVelocityReference(pub);
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY),
                      pub);
      _robot.setVelocityReference(pub);
    }
    if (_torque)
    {
//        std::cout << "torque" << std::endl;
//        std::cout << _command.get(mwoibn::robot_class::INTERFACE::TORQUE) << std::endl;
      //pub.setZero(_dofs); // maybe remove?
      _robot.getEffortReference(pub);
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE),
                      pub);
      _robot.setEffortReference(pub);
    }
    
      _robot.setStiffness(stiffness);
      _robot.setDamping(damping);
    return true;
}
