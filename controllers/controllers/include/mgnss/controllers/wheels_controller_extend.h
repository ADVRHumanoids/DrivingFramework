#ifndef __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H
#define __MGNSS_CONTROLLERS_WHEELS_CONTROLLER_EXTEND_H

#include <mgnss/controllers/wheels_controller.h>
#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>

namespace mgnss
{

namespace controllers {

class WheelsControllerExtend: public WheelsController
{

public:
  WheelsControllerExtend(mwoibn::robot_class::Robot& robot);

  ~WheelsControllerExtend() {}

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

  virtual bool isDoneSteering(const double eps) const
  {
    return _isDone(*_leg_steer_ptr, eps);
  }
  virtual bool isDonePlanar(const double eps) const
  {
    return _isDone(*_steering_ptr, eps);
  }
  virtual bool isDoneWheels(const double eps) const
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
  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;

  void _createAngleTasks();
  void _setInitialConditions();

};
}
}
#endif // WHEELED_MOTION_H
