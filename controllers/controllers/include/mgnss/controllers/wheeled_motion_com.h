#ifndef __MGNSS_CONTROLLERS_WHEELED_MOTION_COM_H
#define __MGNSS_CONTROLLERS_WHEELED_MOTION_COM_H

#include <mgnss/controllers/wheels_controller.h>
#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>

namespace mgnss
{

namespace controllers
{

class WheeledMotionCom: public mgnss::controllers::WheelsController
{

public:
  WheeledMotionCom(mwoibn::robot_class::Robot& robot);

  ~WheeledMotionCom() {}

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

  virtual void updateBase(){

    stepBase();
//    std::cout << "_heading\t" << _heading << std::endl;

    _pelvis_position_ptr->setReference(0, _position);
    _com_ptr->setReference(_position.head(2));
    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate())*_orientation;

    _pelvis_orientation_ptr->setReference(
        0, mwoibn::Quaternion::fromAxisAngle(_z, _heading) * _orientation);
  }

  virtual void fullUpdate(const mwoibn::VectorN& support);
  virtual void compute();

  virtual bool isRunning() { return _robot.isRunning(); }

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
protected:

  std::unique_ptr<mwoibn::hierarchical_control::CenterOfMassTask>
      _com_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;


  virtual void _setInitialConditions();
  virtual void _createTasks();
  virtual void _initIK();

  virtual void _correct();

};
}
}

#endif // WHEELED_MOTION_H
