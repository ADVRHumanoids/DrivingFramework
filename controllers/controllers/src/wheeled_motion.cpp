#include <mgnss/controllers/wheeled_motion.h>

mwoibn::WheeledMotion::WheeledMotion(mwoibn::robot_class::Robot& robot)
    : _robot(robot)
{

  _robot.wait();
  _robot.get();
  _robot.updateKinematics();
  // Set-up hierachical controller
  //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);
  _constraints_ptr.reset(
      new mwoibn::hierarchical_control::ConstraintsTask(_robot));
  mwoibn::Vector3 pelvis;
  pelvis << 1, 1, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                     robot.getLinks("base"));
  _pelvis_position_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSelectiveTask(pelvis_ph,
                                                               pelvis));
  _pelvis_orientation_ptr.reset(
      new mwoibn::hierarchical_control::OrientationSelectiveTask(
          mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                      robot.getLinks("base")),
          pelvis, _robot));
  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask(
          mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                   robot.getLinks("wheels")),
          _robot));

  mwoibn::VectorN selection(12);
  selection << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;

  _leg_z_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                  robot.getLinks("camber")),
      selection, _robot));

  selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

  _leg_xy_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                  robot.getLinks("camber")),
      selection, _robot));

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

  mwoibn::VectorN init_steerings;
  init_steerings.setZero(4);

  _steering_ref_ptr.reset(
      new mgnss::events::Steering(_robot, *_steering_ptr, init_steerings, 0.7, 0.3, _dt, 0.1));

  steerings.resize(_leg_z_ptr->points().size());
  axis << 0, 0, 1;

  for (int i = 0; i < _leg_z_ptr->points().size(); i++)
  {
    steerings[i] = 0;
    _leg_xy_ptr->setReference(i,
                              _leg_xy_ptr->getOffset(i) *
                                  mwoibn::Quaternion::fromAxisAngle(axis, 0.0));
  }


  _select_steer = robot.getDof(robot.getLinks("camber"));
  _l_limits.setZero(_select_steer.size());
  _u_limits.setZero(_select_steer.size());
//  _test_limits.setZero(_leg_steer_ptr->size());
  robot.lower_limits.get(_l_limits, _select_steer);
  robot.upper_limits.get(_u_limits, _select_steer);


  _pelvis_state = _pelvis_position_ptr->points().getPointStateWorld(0);
  _pelvis_position_ptr->setReference(_pelvis_state);
  _heading = _steering_ptr->getState()[2];
  _previous_command = mwoibn::VectorN::Zero(3);
  _command.setZero(_robot.getDofs());
}

void mwoibn::WheeledMotion::nextStep(const mwoibn::VectorN& support,
                                     const mwoibn::Vector3& velocity,
                                     const double omega)
{

  updateBase(velocity, omega);
  updateSupport(support);

  _next_step[0] =
      (_pelvis_state[0] - _steering_ptr->getState()[0]) / _robot.rate();
  _next_step[1] =
      (_pelvis_state[1] - _steering_ptr->getState()[1]) / _robot.rate();
  _next_step[2] =
      (_heading - _steering_ptr->getState()[2]); // just limit the difference

  _next_step[2] -= 6.28318531 * std::floor((_next_step[2] + 3.14159265) /
                                           6.28318531); // limit -pi:pi
  _next_step[2] = _next_step[2] / _robot.rate();
  //  _next_step[0] = velocity[0];
  //  _next_step[1] = velocity[1];
  //  _next_step[2] = omega;
  steering();
}

void mwoibn::WheeledMotion::resetSteering()
{
  for (int i = 0; i < 4; i++)
  {
    _leg_z_ptr->setReference(i, _leg_z_ptr->getOffset(i) *
                                    mwoibn::Quaternion::fromAxisAngle(axis, 0));
  }
}

double mwoibn::WheeledMotion::limit(const double th)
{
  return th - 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

void mwoibn::WheeledMotion::update(const mwoibn::VectorN& support,
                                   const mwoibn::Vector3& velocity,
                                   const double omega)
{

  nextStep(support, velocity, omega);
  compute();
}

void mwoibn::WheeledMotion::fullUpdate(const mwoibn::VectorN& support,
                                       const mwoibn::Vector3& velocity,
                                       const double omega)
{
  _robot.get();
  _robot.updateKinematics();

  update(support, velocity, omega);

  _robot.send();
  _robot.wait();
}
void mwoibn::WheeledMotion::compute()
{

  _command.noalias() = _hierarchical_controller.update();

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::VELOCITY);

  _command.noalias() = _command * _robot.rate();
  _command.noalias() +=
      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::POSITION);
}

void mwoibn::WheeledMotion::steering()
{

  _steering_ref_ptr->compute(_next_step);

  steerings.noalias() = _steering_ref_ptr->get();
//  std::cout << "steerings\t";
//  std::cout << steerings.transpose();
  for (int i = 0; i < 4; i++)
  {
    steerings[i] = (steerings[i] < _l_limits[i]) ? steerings[i] + mwoibn::PI : steerings[i];
    steerings[i] = (steerings[i] > _u_limits[i]) ? steerings[i] - mwoibn::PI : steerings[i];
    _leg_z_ptr->setReference(
        i, _leg_z_ptr->getOffset(i) *
               mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
  }
  _steering_ref_ptr->set(steerings);
//  std::cout << steerings.transpose() << std::endl;
}
