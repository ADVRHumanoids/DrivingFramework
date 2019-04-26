#include "mwoibn/communication_modules/ros_controller.h"

bool mwoibn::communication_modules::RosController::run()
{

  // std::cout << "run" << std::endl;
  if(!_initialized) {initialize();}

  if (_position)
  {/*
    if(_filter){
      mapTo(_command.position.get, _filtered);
      _position_filter_ptr->update(_filtered);
      _command.position.set(_filtered, _map.reversed());
    }
    */
    mapTo(_command.position.get(), _des_q.position);
    //des_q.position = pub_qq;
  }
  if (_velocity)
  {
    /*
    if(_filter){
      mapTo(_command.velocity.get, _filtered);
      _velocity_filter_ptr->update(_filtered);
      _command.velocity.set(_filtered, _map.reversed());
    }
    */
    mapTo(_command.velocity.get(), _des_q.velocity);
    //des_q.velocity = pub_qq;
  }

  if (_torque)
  {
    /*
    if(_filter){
      mapTo(_command.torque.get, _filtered);
      _torque_filter_ptr->update(_filtered);
      _command.torque.set(_filtered, _map.reversed());
    }
    */
    mapTo(_command.torque.get(), _des_q.effort);
    // std::cout << _command.torque.get().transpose() << std::endl;
    //des_q.effort = pub_qq;
  }

  // des_q.header.stamp = ros::Time::now();
  _command_pub.publish(_des_q);
  // std::cout << "publish" << std::endl;

  return _initialized;
}
