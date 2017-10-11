#include <mgnss/controllers/online_centralized_controller.h>

mgnss::controllers::OnlineCentralizedController::OnlineCentralizedController(mwoibn::robot_class::Robot& robot): _robot(robot){

_dynamic_model_ptr.reset(
    new mwoibn::dynamic_models::QrDecomposition(_robot)); // offline
_gravity_compensation_ptr.reset(
    new mwoibn::gravity_compensation::SimpleQRGravityCompensation(
        *_dynamic_model_ptr, _robot));

_actuation_model_ptr.reset(new mwoibn::motor_side_reference::SeaReference(
    _robot, *_gravity_compensation_ptr));

}

void mgnss::controllers::OnlineCentralizedController::update(){

  _gravity_compensation_ptr->update();
   if(_motor_side)
    _actuation_model_ptr->update();

}

void mgnss::controllers::OnlineCentralizedController::fullUpdate(const mwoibn::VectorN& reference){

  if(!_robot.get()) return;

  _robot.updateKinematics();

  _robot.command.set(reference,
      mwoibn::robot_class::INTERFACE::POSITION);

  update();

  _robot.send();
  _robot.wait();
}

void mgnss::controllers::OnlineCentralizedController::fullUpdate(const mwoibn::VectorN& reference, const mwoibn::VectorN& vel_reference){

  if(!_robot.get()) return;

  _robot.updateKinematics();

  _robot.command.set(reference,
      mwoibn::robot_class::INTERFACE::POSITION);

  _robot.command.set(vel_reference,
      mwoibn::robot_class::INTERFACE::VELOCITY);

  update();

  _robot.send();
  _robot.wait();
}
