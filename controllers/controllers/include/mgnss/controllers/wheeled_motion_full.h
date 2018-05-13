#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_FULL_H
#define __MGNSS_CONTROLLERS__WHEELED_MOTION_FULL_H

#include <mgnss/controllers/wheels_controller.h>
#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>

namespace mgnss
{

namespace controllers {

class WheeledMotionFull: public WheelsController
{

public:
  WheeledMotionFull(mwoibn::robot_class::Robot& robot);

  ~WheeledMotionFull() {}


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

    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate())*_orientation;

    _pelvis_orientation_ptr->setReference(
        0, mwoibn::Quaternion::fromAxisAngle(_z, _heading) * _orientation);
  }

  virtual double getBaseGroundX()
  {
    return _pelvis_position_ptr->points().getPointStateWorld(0)[0];
  }
  virtual double getBaseGroundY()
  {
    return _pelvis_position_ptr->points().getPointStateWorld(0)[1];
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

protected:

  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;

  mwoibn::VectorN _test_limits;

  virtual void _setInitialConditions();
  virtual void _createTasks();
  virtual void _initIK();

};
}
}

#endif // WHEELED_MOTION_H
