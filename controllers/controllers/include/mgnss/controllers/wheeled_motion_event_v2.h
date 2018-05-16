#ifndef __MGNSS_CONTROLLERS_WHEELS_MOTION_EVENT_2_H
#define __MGNSS_CONTROLLERS_WHEELS_MOTION_EVENT_2_H

#include <mwoibn/hierarchical_control/center_of_mass_task_v2.h>
#include <mgnss/controllers/wheels_controller_extend.h>

namespace mgnss
{

namespace controllers
{

class WheeledMotionEvent2: public WheelsControllerExtend
{

public:
  WheeledMotionEvent2(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& full_robot);

  ~WheeledMotionEvent2() {}

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

  mwoibn::VectorN  _test_limits;

  virtual void _setInitialConditions();
  virtual void _createTasks();
  virtual void _initIK();
};
}
}

#endif // WHEELED_MOTION_H
