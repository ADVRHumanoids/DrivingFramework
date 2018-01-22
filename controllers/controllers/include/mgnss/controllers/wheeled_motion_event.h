#ifndef PROGRAMS_WHEELED_MOTION_EVENT_H
#define PROGRAMS_WHEELED_MOTION_EVENT_H

#include <mwoibn/robot_class/robot.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/constraints_task.h>

#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v5.h>
#include <mgnss/controllers/steering_v3.h>

#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>
#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>

namespace mwoibn
{

class WheeledMotionEvent
{

public:
  WheeledMotionEvent(mwoibn::robot_class::Robot& robot);

  ~WheeledMotionEvent() {}

  void init();

  void stop();

  void setRate(){ _dt = _robot.rate(); _steering_ref_ptr->setRate(_dt);}

  void resetSteering();
  void resteer(int i){_resteer[i] = true;
                     std::cout << "started resteering" << std::endl;}


  void setSteering(int i, double th)
  {
    _leg_steer_ptr->setReference(i, th);
  }
  void setCastor(int i, double th)
  {
    _leg_castor_ptr->setReference(i, th);
  }
  void setCamber(int i, double th)
  {
    _leg_camber_ptr->setReference(i, th);
  }

  void rotateBaseX(double th)
  {
    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, th)*_orientation;
  }

  void rotateBaseY(double th)
  {
    _orientation = mwoibn::Quaternion::fromAxisAngle(_y, th)*_orientation;
  }

  void setBaseRotVelX(double dth){
    _angular_vel[0] = dth;
  }
  void setBaseRotVelY(double dth){
    _angular_vel[1] = dth;
  }

  void setBaseX(double x){
    _position[0] = x;
  }
  void setBaseY(double y){
    _position[1] = y;
  }
  void setBaseZ(double z){
    _position[2] = z;
  }
  void setBaseHeading(double th){
    _heading = th;
  }

  void setBaseDotX(double dx){
    _linear_vel[0] = dx;
  }
  void setBaseDotY(double dy){
    _linear_vel[1] = dy;
  }
  void setBaseDotZ(double dz){
    _linear_vel[2] = dz;
  }
  void setBaseDotHeading(double th){
    _angular_vel[2] = th;
  }

  mwoibn::VectorN getSteering() { return steerings; }

  void updateSupport(const mwoibn::VectorN& support)
  {
    _steering_ptr->setReference(support);
  }

  void updateBase(){

//    std::cout << "_linear_vel\t" << _linear_vel << std::endl;
//    std::cout << "_rate\t" << _robot.rate() << std::endl;
//    std::cout << "_angular_vel\t" << _angular_vel << std::endl;

    _position += _linear_vel * _robot.rate();
    _heading += _angular_vel[2] * _robot.rate();
    _heading -= 6.28318531 * std::floor((_heading + 3.14159265) /
                                        6.28318531); // limit -pi:pi

//    std::cout << "_heading\t" << _heading << std::endl;

    _pelvis_position_ptr->setReference(0, _position);
    _com_ptr->setReference(_position.head<2>());
    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate())*_orientation;

    _pelvis_orientation_ptr->setReference(
        0, mwoibn::Quaternion::fromAxisAngle(_z, _heading) * _orientation);
  }

  void steering();

  void fullUpdate(const mwoibn::VectorN& support);
  void compute();

  void nextStep(const mwoibn::VectorN& support);

  void update(const mwoibn::VectorN& support);

  double limit(const double th);

  bool isRunning() { return _robot.isRunning(); }

  bool isDonePosition(const double eps)
  {
    return _isDone(*_pelvis_position_ptr, eps);
  }
  bool isDoneOrientation(const double eps)
  {
    return _isDone(*_pelvis_orientation_ptr, eps);
  }
  bool isDoneSteering(const double eps) const
  {
    return _isDone(*_leg_steer_ptr, eps);
  }
  bool isDonePlanar(const double eps) const
  {
    return _isDone(*_steering_ptr, eps);
  }
//  bool isDoneWheels(const double eps) const
//  {
//    return _isDone(*_leg_castor_ptr, eps);
//  }

  const mwoibn::VectorN& getSupportReference()
  {
    return _steering_ptr->getReference();
  }
  const mwoibn::VectorN& getBodyPosition()
  {
    return _pelvis_position_ptr->getReference();
  }

  void claim(int i){
    _steering_ptr->claimContact(i);
    _constraints_ptr->claimContact(i);
  }

  void release(int i){
    _steering_ptr->releaseContact(i);
    _constraints_ptr->releaseContact(i);
  }


protected:
  bool _isDone(mwoibn::hierarchical_control::ControllerTask& task,
               const double eps) const
  {
    return task.getError().cwiseAbs().maxCoeff() < eps;
  }
  mwoibn::robot_class::Robot& _robot;

  std::unique_ptr<mwoibn::hierarchical_control::ConstraintsTask>
      _constraints_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CartesianSelectiveTask>
      _pelvis_position_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::OrientationSelectiveTask>
      _pelvis_orientation_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CenterOfMassTask>
      _com_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CartesianFlatReferenceTask2>
      _steering_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;

  std::unique_ptr<mgnss::events::Steering3> _steering_ref_ptr;

  mwoibn::hierarchical_control::HierarchicalController _hierarchical_controller;

  double rate = 200;
  double _dt, orientation = 0, _heading;
  mwoibn::VectorN steerings, _command, _previous_command;
  mwoibn::Vector3 _next_step, _position, _angular_vel, _linear_vel;
  mwoibn::Axis _x, _y, _z;
  mwoibn::Quaternion _orientation;
  bool _reference = false;
  mwoibn::VectorInt _select_steer, _select_wheel;
  mwoibn::VectorN _l_limits, _u_limits, _test_steer, _test_wheel;
  int count = 0;

  mwoibn::VectorBool _resteer;

};
}

#endif // WHEELED_MOTION_H
