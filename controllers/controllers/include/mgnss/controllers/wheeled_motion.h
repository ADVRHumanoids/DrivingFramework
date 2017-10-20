#ifndef PROGRAMS_WHEELED_MOTION_H
#define PROGRAMS_WHEELED_MOTION_H

#include <mwoibn/robot_class/robot.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/constraints_task.h>

#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v3.h>
#include <mgnss/controllers/steering.h>

#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>


namespace mwoibn
{

class WheeledMotion
{

public:
  WheeledMotion(mwoibn::robot_class::Robot& robot);

  ~WheeledMotion() {}

  void init()
  {
    _steering_ptr->init();
  }
  void resetSteering();

  void updateSupport(const mwoibn::VectorN& support)
  {
//    for (int i = 0; i < 4; i++)
//      _steering_ptr->setReference(i, support.segment<2>(2 * i));
    _steering_ptr->setReference(support);
  }

/*  void updateBase(const mwoibn::Vector3& pose, const double heading)
  {
    
    position.head(2) = pose.head(2);
    position[2] = heading;

    _pelvis_position_ptr->setReference(0, pose);

    _pelvis_orientation_ptr->setReference(
        0, _pelvis_orientation_ptr->getOffset(0) *
               mwoibn::Quaternion::fromAxisAngle(axis, heading));
  }*/

    void updateBase(const mwoibn::Vector3& velocity, const double omega)
  {

    for(int i = 0; i < _pelvis_state.size(); i++)
        if(_previous_command[i] != velocity[i]) _pelvis_state[i] = _pelvis_position_ptr->points().getPointStateWorld(0)[i];
    _pelvis_state += velocity*_robot.rate();
    //_pelvis_state = _pelvis_position_ptr->points().getPointStateWorld(0) + 
    _heading += omega * _robot.rate();


    _pelvis_position_ptr->setReference(0, _pelvis_state);

    _pelvis_orientation_ptr->setReference(
        0, _pelvis_orientation_ptr->getOffset(0) *
               mwoibn::Quaternion::fromAxisAngle(axis, _heading));
  }

  
  void steering();

  void fullUpdate(const mwoibn::VectorN& support, const mwoibn::Vector3& velocity,
                  const double omega);
  void compute();

  void nextStep(const mwoibn::VectorN& support, const mwoibn::Vector3& velocity, const double omega);

  void update(const mwoibn::VectorN& support, const mwoibn::Vector3& velocity, const double omega);

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
  bool isDoneSteering(const double eps) const { return _isDone(*_leg_z_ptr, eps); }
  bool isDonePlanar(const double eps) const { return _isDone(*_steering_ptr, eps); }
  bool isDoneWheels(const double eps) const { return _isDone(*_leg_xy_ptr, eps); }

  const mwoibn::VectorN& getSupportReference()
  {
    return _steering_ptr->getReference();
  }
  const mwoibn::VectorN& getBodyPosition()
  {
    return _pelvis_position_ptr->getReference();
  }

protected:
  bool _isDone(mwoibn::hierarchical_control::ControllerTask& task, const double eps) const
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

  std::unique_ptr<mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask>
      _steering_ptr;

  std::unique_ptr<mwoibn::hierarchical_control::OrientationSelectiveTask>
      _leg_xy_ptr;
  std::unique_ptr<mwoibn::hierarchical_control::OrientationSelectiveTask>
      _leg_z_ptr;

  std::unique_ptr<events::Steering> _steering_ref_ptr;

  mwoibn::hierarchical_control::HierarchicalController _hierarchical_controller;

  double rate = 200;
  double _dt, orientation = 0, _heading;
  mwoibn::VectorN steerings, _command, _previous_command;
  mwoibn::Vector3 axis, _next_step, _pelvis_state;
  bool _reference = false;

};
}

#endif // WHEELED_MOTION_H
