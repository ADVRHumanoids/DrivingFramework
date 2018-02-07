#include "mwoibn/communication_modules/ros_controller.h"

bool mwoibn::communication_modules::RosController::send()
{

  if(!_initialized) {_initFilters(); _initialized = true;}

  if (_position)
  {
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), _filtered);
      _position_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::POSITION);
    }

    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), _des_q.position);
  }
  if (_velocity)
  {
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), _filtered);
      _velocity_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::VELOCITY);
    }
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), _des_q.velocity);
  }

  if (_torque)
  {
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _filtered);
      _torque_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::TORQUE);
    }
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _des_q.effort);

  }

  _command_pub.publish(_des_q);

  return true;
}
