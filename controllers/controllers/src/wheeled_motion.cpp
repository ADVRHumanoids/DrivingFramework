#include <mgnss/controllers/wheeled_motion.h>

mwoibn::WheeledMotion::WheeledMotion(mwoibn::robot_class::Robot& robot)
    : _robot(robot)
{

  _robot.update();
  // Set-up hierachical controller
  //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);
  _constraints_ptr.reset(
      new mwoibn::hierarchical_control::ConstraintsTask(_robot));
  mwoibn::Vector3 pelvis;
  pelvis << 1, 1, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                     {"pelvis"});
  _pelvis_position_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSelectiveTask(pelvis_ph,
                                                               pelvis));
  _pelvis_orientation_ptr.reset(
      new mwoibn::hierarchical_control::OrientationSelectiveTask(
          mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                      {"pelvis"}),
          pelvis, _robot));
  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask(
          mwoibn::point_handling::PositionsHandler(
              "ROOT", _robot, std::vector<std::string>{"wheel_1", "wheel_2",
                                                       "wheel_3", "wheel_4"}),
          _robot));

  mwoibn::VectorN selection(12);
  selection << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;

  _leg_z_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler(
          "ROOT", _robot, std::vector<std::string>{"ankle2_1", "ankle2_2",
                                                   "ankle2_3", "ankle2_4"}),
      selection, _robot));

  selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

  _leg_xy_ptr.reset(new mwoibn::hierarchical_control::OrientationSelectiveTask(
      mwoibn::point_handling::OrientationsHandler(
          "ROOT", _robot, std::vector<std::string>{"ankle2_1", "ankle2_2",
                                                   "ankle2_3", "ankle2_4"}),
      selection, _robot));

  int task = 0;
  int ratio = 1;
  double damp = 1e-4;
  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  _hierarchical_controller.addTask(_constraints_ptr.get(), gain, task, damp);
  task++;
  gain << 1000 * ratio;
  _hierarchical_controller.addTask(_leg_z_ptr.get(), gain, task, damp);
  task++;
  gain << 1000 * ratio;
  _hierarchical_controller.addTask(_pelvis_position_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 500 * ratio;
  _hierarchical_controller.addTask(_pelvis_orientation_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 300 * ratio;
  _hierarchical_controller.addTask(_steering_ptr.get(), gain, task, damp);
  task++;
  gain << 1000 * ratio;
  _hierarchical_controller.addTask(_leg_xy_ptr.get(), gain, task, 1e-3);
  task++;


  _dt = 1 / rate;

  _steering_ref_ptr.reset(new events::Steering(_robot, *_steering_ptr, 0.3, 0.7,
                                               _dt));

  steerings.resize(_leg_z_ptr->points().size());
  axis << 0, 0, 1;

  for (int i = 0; i < _leg_z_ptr->points().size(); i++)
  {
    steerings[i] = 0;
    _leg_xy_ptr->setReference(i,
                              _leg_xy_ptr->getOffset(i) *
                                  mwoibn::Quaternion::fromAxisAngle(axis, 0.0));
  }
  _command.setZero(_robot.getDofs());
}

void mwoibn::WheeledMotion::nextStep(const mwoibn::VectorN& support,
                                     const mwoibn::Vector3& pose, const double heading)
{

  updateBase(pose, heading);
  updateSupport(support);

  _next_step = position - _steering_ptr->getState().head(3);

  _next_step[2] = limit(_next_step[2]);
  _next_step = _next_step / _dt;

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
                                   const mwoibn::Vector3& pose, const double heading)
{

  nextStep(support, pose, heading);
  compute();
}

void mwoibn::WheeledMotion::fullUpdate(const mwoibn::VectorN& support,
                                       const mwoibn::Vector3& pose, const double heading)
{
  _robot.get();
  _robot.updateKinematics();

  update(support, pose, heading);

  _robot.send();
  _robot.wait();
}
void mwoibn::WheeledMotion::compute()
{

  _command.noalias() = _hierarchical_controller.update() * _dt;

  _command.noalias() += _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::POSITION);

}

void mwoibn::WheeledMotion::steering()
{

//  events::combined2(_robot, *_steering_ptr, steerings, next_step, gain1, gain2,
//                    _dt);

  _steering_ref_ptr->compute(_next_step);

  steerings.noalias() = _steering_ref_ptr->get();

  for (int i = 0; i < 4; i++)
  {
    _leg_z_ptr->setReference(
        i, _leg_z_ptr->getOffset(i) *
               mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
  }
}
