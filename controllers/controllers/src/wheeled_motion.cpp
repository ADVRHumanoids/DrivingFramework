#include <mgnss/controllers/wheeled_motion.h>
#include <mgnss/controllers/steering_v4.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v3.h>

mgnss::controllers::WheeledMotion::WheeledMotion(mwoibn::robot_class::Robot& robot)
    : mgnss::controllers::WheelsController(robot)
{

  _robot.wait();
  _robot.get();
  _robot.updateKinematics();

  _createTasks();
  _initIK();

  mwoibn::VectorN init_steerings;
  init_steerings.setZero(4);

  _steering_ref_ptr.reset(
      new mgnss::events::Steering4(_robot, *_steering_ptr, init_steerings, 0.7, 0.3, _dt, 0.1));

  _allocate();
  init();
}

void mgnss::controllers::WheeledMotion::_setInitialConditions(){

  _steering_ptr->init();

  for (int i = 0; i < _leg_z_ptr->points().size(); i++)
  {
    steerings[i] = 0;
    _leg_xy_ptr->setReference(i,
                              _leg_xy_ptr->getOffset(i)*(
                                  mwoibn::Quaternion::fromAxisAngle(_z, 0.0)));
  }

  _position = _pelvis_position_ptr->points().getPointStateWorld(0);
  _pelvis_position_ptr->setReference(_position);
  _heading = _steering_ptr->getState()[2];
}


void mgnss::controllers::WheeledMotion::_createTasks(){

  _constraints_ptr.reset(
      new mwoibn::hierarchical_control::ConstraintsTask(_robot));
  mwoibn::Vector3 pelvis;
  pelvis << 1, 1, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                     _robot.getLinks("base"));
  _pelvis_position_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSelectiveTask(pelvis_ph,
                                                               pelvis));
  _pelvis_orientation_ptr.reset(
      new mwoibn::hierarchical_control::OrientationSelectiveTask(
          mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                      _robot.getLinks("base")),
          pelvis, _robot));
  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask(
          mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                   _robot.getLinks("wheels")),
          _robot));

  mwoibn::VectorN selection(12);
  selection << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;

  _leg_z_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                  _robot.getLinks("camber")),
      selection, _robot));

  selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

  _leg_xy_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                  _robot.getLinks("camber")),
      selection, _robot));

}

void mgnss::controllers::WheeledMotion::_initIK(){

  int task = 0;
  int ratio = 1;
  double damp = 1e-4;
  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  _hierarchical_controller.addTask(_constraints_ptr.get(), gain, task, damp);
  task++;
  gain << 25 * ratio;
  _hierarchical_controller.addTask(_leg_z_ptr.get(), gain, task, damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller.addTask(_pelvis_orientation_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller.addTask(_pelvis_position_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller.addTask(_steering_ptr.get(), gain, task, damp);
  task++;
  gain << 50 * ratio;
  _hierarchical_controller.addTask(_leg_xy_ptr.get(), gain, task, 0.1);
  task++;

  _dt = _robot.rate();

}

void mgnss::controllers::WheeledMotion::resetSteering()
{
  for (int i = 0; i < 4; i++)
  {
    _leg_z_ptr->setReference(i, _leg_z_ptr->getOffset(i)*(
                                    mwoibn::Quaternion::fromAxisAngle(_z, 0)));
  }
}

void mgnss::controllers::WheeledMotion::update(const mwoibn::VectorN& support,
                                   const mwoibn::Vector3& velocity,
                                   const double omega)
{
  _linear_vel = velocity;
  _angular_vel[2] = omega;

  updateSupport(support);

  nextStep();
  compute();
}

void mgnss::controllers::WheeledMotion::fullUpdate(const mwoibn::VectorN& support)
{
  _robot.get();
  _robot.updateKinematics();

  update(support);

  _robot.send();
  _robot.wait();
}

void mgnss::controllers::WheeledMotion::fullUpdate(const mwoibn::VectorN& support,
                                       const mwoibn::Vector3& velocity,
                                       const double omega)
{
  _linear_vel = velocity;
  _angular_vel[2] = omega;

  update(support);

}

void mgnss::controllers::WheeledMotion::steering()
{

  mgnss::controllers::WheelsController::steering();

  _steering_ref_ptr->set(steerings);
}
