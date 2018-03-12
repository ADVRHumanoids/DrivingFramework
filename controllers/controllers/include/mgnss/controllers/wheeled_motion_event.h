#ifndef __MGNSS_WHEELED_MOTION_EVENT_H
#define __MGNSS_WHEELED_MOTION_EVENT_H

//#include <mwoibn/robot_class/robot.h>
#include <mgnss/modules/base.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/constraints_task.h>

#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v7.h>
#include <mgnss/controllers/steering_v5.h>

#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>
#include <mwoibn/hierarchical_control/castor_angle_task.h>
#include <mwoibn/hierarchical_control/camber_angle_task_2.h>
#include <mwoibn/hierarchical_control/steering_angle_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionEvent: public modules::Base
{

public:
  WheeledMotionEvent(mwoibn::robot_class::Robot& robot, std::string config_file);

  ~WheeledMotionEvent() {}

  virtual void init();

  virtual void stop();
  virtual void send(){
    _robot.send();
  } // NOT IMPLEMENTED

  virtual void update(){
    //std::cout << _steering_ptr->getReference().transpose() << std::endl;
    _robot.centerOfMass().update();

    updateBase();

    _next_step[0] =
        (_position[0] - _robot.centerOfMass().get()[0]) / _robot.rate();
    _next_step[1] =
        (_position[1] - _robot.centerOfMass().get()[1]) / _robot.rate();
    _next_step[2] =
        (_heading - _steering_ptr->getState()[2]); // just limit the difference

    _next_step[2] -= 6.28318531 * std::floor((_next_step[2] + 3.14159265) /
                                             6.28318531); // limit -pi:pi
    _next_step[2] = _next_step[2] / _robot.rate();
    steering();
    compute();

  } // NOT IMPLEMENTED
  virtual void close(){} // NOT IMPLEMENTED
  virtual void setRate(double rate){
    modules::Base::setRate(rate);
    setRate();
  }

  void setRate(){ _dt = _robot.rate(); _steering_ref_ptr->setRate(_dt);}

  void resetSteering();
  void resteer(int i){_resteer[i] = true;
                      _start_steer[i] = _test_steer[i];

                     //std::cout << "started resteering" << std::endl;
                     }
  void stopResteer(int i){_resteer[i] = false;
                      _start_steer[i] = _test_steer[i];

                     //std::cout << "stoped resteering" << std::endl;
                     }

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
//     std::cout << "_angular_vel\t" << _angular_vel << std::endl;

    _position += _linear_vel * _robot.rate();
    _heading += _angular_vel[2] * _robot.rate();
//    std::cout << "before limits heading\t" << _heading << std::endl;

    _heading -= 6.28318531 * std::floor((_heading + 3.14159265) /
                                        6.28318531); // limit -pi:pi

//    std::cout << "_heading\t" << _heading << std::endl;
//    std::cout << "_z\t" << _z << std::endl;

    _com_ref << _position[0], _position[1];
    _pelvis_position_ptr->setReference(0, _position);
    _com_ptr->setReference(_com_ref);

    _orientation = mwoibn::Quaternion::fromAxisAngle(_x, _angular_vel[0]*_robot.rate())*mwoibn::Quaternion::fromAxisAngle(_y, _angular_vel[1]*_robot.rate())*_orientation;


    _pelvis_orientation_ptr->setReference(0, _orientation * mwoibn::Quaternion::fromAxisAngle(_z, _heading));
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
 //   std::cout << "claim\t" << i << std::endl;
    _steering_ptr->claimContact(i);
    _constraints_ptr->claimContact(i);
  }

  void release(int i){
    _steering_ptr->releaseContact(i);
    _constraints_ptr->releaseContact(i);
  }

  mwoibn::VectorN getCom(){ return _robot.centerOfMass().get().head<2>();}
  const mwoibn::Vector3& getComFull(){ return _robot.centerOfMass().get();}
  const mwoibn::VectorN& errorCom(){return _com_ptr->getError();}

  const mwoibn::VectorN& refCom(){return _com_ptr->getReference();}
  double refComX(){return _com_ptr->getReference()(0,0);}
  double refComY(){return _com_ptr->getReference()(0,1);}

  const mwoibn::Vector3& getCp(int i){ return _steering_ptr->getPointStateReference(i);}

  mwoibn::VectorN errorCp(int i){ return _steering_ptr->getReferenceError(i);}
  const mwoibn::VectorN& refCp(){ return _steering_ptr->getReference();}
  const mwoibn::VectorN& getSteer(){ return _leg_steer_ptr->getCurrent();}
  const mwoibn::VectorN& errorSteer(){ return _leg_steer_ptr->getError();}
  const mwoibn::VectorN& refSteer(){ return steerings;}
  const mwoibn::Vector3& getLinVel(){ return _linear_vel;}
  const mwoibn::Vector3& getAngVel(){ return _angular_vel;}
  const mwoibn::VectorN& getSteerICM(){return _steering_ref_ptr->getICM();}
  const mwoibn::VectorN& getSteerSP(){return _steering_ref_ptr->getSP();}
  const mwoibn::VectorN& getVelICM(){return _steering_ref_ptr->vICM();}
  const mwoibn::VectorN& getVelSP(){return _steering_ref_ptr->vSP();}
  const mwoibn::VectorN& getDampingICM(){return _steering_ref_ptr->getDampingICM();}
  const mwoibn::VectorN& getDampingSP(){return _steering_ref_ptr->getDampingSP();}
  const mwoibn::VectorBool& isResteer(){return _resteer;}
  const mwoibn::VectorN& getAnkleYaw(){return _test_steer;}
  mwoibn::VectorN getBase(){return _robot.state.get().head<3>();}
  const mwoibn::VectorN& getBaseError(){return _pelvis_position_ptr->getError();}
  const mwoibn::VectorN& getBaseOrnError(){return _pelvis_orientation_ptr->getError();}

//  bool evenstHandler(custom_services::updatePDGains::Request& req,
//                     custom_services::updatePDGains::Response& res);

protected:
  bool _isDone(mwoibn::hierarchical_control::ControllerTask& task,
               const double eps) const
  {
    return task.getError().cwiseAbs().maxCoeff() < eps;
  }
  //mwoibn::robot_class::Robot& _robot;

  std::unique_ptr<mwoibn::hierarchical_control::ConstraintsTask>
      _constraints_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CartesianSelectiveTask>
      _pelvis_position_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::OrientationSelectiveTask>
      _pelvis_orientation_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CenterOfMassTask>
      _com_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CartesianFlatReferenceTask4>
      _steering_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::CamberAngleTask>
      _leg_camber_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::CastorAngleTask>
      _leg_castor_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::SteeringAngleTask>
      _leg_steer_ptr;

  std::unique_ptr<mgnss::events::Steering5> _steering_ref_ptr;

  mwoibn::hierarchical_control::HierarchicalController _hierarchical_controller;

  double rate = 200;
  double _dt, orientation = 0, _heading;
  mwoibn::VectorN steerings, _command, _previous_command, _com_ref;
  mwoibn::Vector3 _next_step, _position, _angular_vel, _linear_vel;
  mwoibn::Axis _x, _y, _z;
  mwoibn::Quaternion _orientation;
  bool _reference = false;
  mwoibn::VectorInt _select_steer, _select_wheel;
  mwoibn::VectorN _l_limits, _u_limits, _test_steer, _current_steer, _start_steer;
  int count = 0;

  mwoibn::VectorBool _resteer;

};
}
}

#endif // WHEELED_MOTION_H
