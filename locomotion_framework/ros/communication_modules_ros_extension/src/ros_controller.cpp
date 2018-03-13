#include "mwoibn/communication_modules/ros_controller.h"

bool mwoibn::communication_modules::RosController::send()
{

  if(!_initialized) {initialize();}

  if (_position)
  {/*
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), _filtered);
      _position_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::POSITION);
    }
    */
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::POSITION), _des_q.position);
    //des_q.position = pub_qq;
  }
  if (_velocity)
  {
    /*
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), _filtered);
      _velocity_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::VELOCITY);
    }
    */
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::VELOCITY), _des_q.velocity);
    //des_q.velocity = pub_qq;
  }

  if (_torque)
  {
    /*
    if(_filter){
      mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _filtered);
      _torque_filter_ptr->update(_filtered);
      _command.set(_filtered, _map.reversed(), mwoibn::robot_class::INTERFACE::TORQUE);
    }
    */
    mapTo(_command.get(mwoibn::robot_class::INTERFACE::TORQUE), _des_q.effort);
    //des_q.effort = pub_qq;
  }

  // des_q.header.stamp = ros::Time::now();
  _command_pub.publish(_des_q);

  return _initialized;
}
