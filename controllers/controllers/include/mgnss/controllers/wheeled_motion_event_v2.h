#ifndef __MGNSS_CONTROLLERS_WHEELS_MOTION_EVENT_2_H
#define __MGNSS_CONTROLLERS_WHEELS_MOTION_EVENT_2_H

#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task_v2.h>
#include <mgnss/controllers/wheels_controller.h>

namespace mgnss
{

namespace controllers
{

class WheeledMotionEvent2: public WheelsController
{

public:
  WheeledMotionEvent2(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& full_robot);

  ~WheeledMotionEvent2() {}

  void resetSteering();

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

  void setBaseX(double x){
    _position[0] = x;
  }
  void setBaseY(double y){
    _position[1] = y;
  }
  void setBaseZ(double z){
    _position[2] = z;
  }

  void updateBase(){

    stepBase();

    _pelvis_position_ptr->setReference(0, _position);
    _com_ptr->setReference(_position.head(2));
    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate())*_orientation;

    _pelvis_orientation_ptr->setReference(
        0, mwoibn::Quaternion::fromAxisAngle(_z, _heading) * _orientation);
  }

  virtual double getBaseGroundX()
  {
    return _robot.centerOfMass().get()[0];
  }
  virtual double getBaseGroundY()
  {
    return _robot.centerOfMass().get()[1];
  }
  virtual double getBaseGroundZ()
  {
    return _pelvis_position_ptr->points().getPointStateWorld(0)[2];
  }
  virtual double getBaseGroundRz(){
    return _steering_ptr->getState()[2];}

  void fullUpdate(const mwoibn::VectorN& support);

  bool isDoneSteering(const double eps) const
  {
    return _isDone(*_leg_steer_ptr, eps);
  }
  bool isDonePlanar(const double eps) const
  {
    return _isDone(*_steering_ptr, eps);
  }
  bool isDoneWheels(const double eps) const
  {
    return _isDone(*_leg_camber_ptr, eps);
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

  mwoibn::robot_class::Robot& _full_robot;

  std::unique_ptr<mwoibn::hierarchical_control::CenterOfMassTask2>
      _com_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;

  mwoibn::VectorN  _test_limits;

  virtual void _setInitialConditions();
  virtual void _createTasks();
  virtual void _initIK();
};
}
}

#endif // WHEELED_MOTION_H
