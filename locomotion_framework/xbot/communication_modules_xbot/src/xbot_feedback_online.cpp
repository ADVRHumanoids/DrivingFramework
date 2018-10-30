#include "mwoibn/communication_modules/xbot_feedback_online.h"

bool mwoibn::communication_modules::XBotFeedbackOnline::get()
{

  if (!_initialized) initialize();

  if (_position){
    _robot.getMotorPosition(_pub);
    _command.position.set(_pub, _map.reversed());
  }
  if (_velocity){
    _robot.getJointVelocity(_pub);
    _command.velocity.set(_pub, _map.reversed());
  }
  if (_torque){
    _robot.getJointEffort(_pub);
    _command.torque.set(_pub, _map.reversed());
  }

  return _initialized;
}

bool mwoibn::communication_modules::XBotFeedbackOnline::_inLimits(mwoibn::robot_class::INTERFACE interface){
  bool success = true;
  for (int i = 0; i < _command.interface(interface).size(); i++)
  {
    success = _inLimits(i, interface) && success;
  }
  return success;
}

bool mwoibn::communication_modules::XBotFeedbackOnline::_inLimits(int i, mwoibn::robot_class::INTERFACE interface){

    if(_map.get()[i] == mwoibn::NON_EXISTING) return true;

    if (_lower_limits.interface(interface).get(i) == mwoibn::NON_EXISTING)
       return true;

    bool in_limits = _command.interface(interface).get(i) > _lower_limits.interface(interface).get(i);
    in_limits = _command.interface(interface).get(i) < _upper_limits.interface(interface).get(i) && in_limits;


    if(!in_limits)
      std::cout << "Encoder reading for " << _robot.getJointByDofIndex(_map.get()[i])->getJointName()
                << " is outside defined joint limits.\n"
                << "\t reading: " << _command.interface(interface).get(i)
                << "\n\t lower limit" << _lower_limits.interface(interface).get(i)
                << "\n\t upper limit" << _upper_limits.interface(interface).get(i);
    return in_limits;

}

bool mwoibn::communication_modules::XBotFeedbackOnline::reset(){
  bool success = true;

  if(_position){
    success = _inLimits(mwoibn::robot_class::INTERFACE::POSITION) && success;
  }
  if(_velocity){
    success = _inLimits(mwoibn::robot_class::INTERFACE::VELOCITY) && success;
  }
  if(_torque){
    success = _inLimits(mwoibn::robot_class::INTERFACE::TORQUE) && success;
  }
  return success;
}

void mwoibn::communication_modules::XBotFeedbackOnline::_initLimit(mwoibn::robot_class::State& limits, double tolerance, mwoibn::robot_class::INTERFACE interface){

    int size = limits.interface(interface).size();

    mwoibn::VectorN temp_limits = tolerance*mwoibn::VectorN::Ones(size);
    for (int i = 0; i < size; i++){
            if(limits.interface(interface).get(i) == mwoibn::NON_EXISTING)
                temp_limits[i] = mwoibn::NON_EXISTING;
            else
                temp_limits[i] += limits.interface(interface).get(i);
    }
    limits.interface(interface).set(temp_limits);
}
