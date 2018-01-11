#include <mgnss/controllers/wheeled_motion_event_v2.h>

mwoibn::WheeledMotionEvent2::WheeledMotionEvent2(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& full_robot)
    : _robot(robot), _full_robot(full_robot)
{
  _x << 1, 0, 0;
  _y << 0, 1, 0;
  _z << 0, 0, 1;

  _robot.wait();
  _robot.get();
  _robot.updateKinematics();

  _full_robot.get();
  _full_robot.updateKinematics();

  _full_robot.centerOfMass().update(true);
  // Set-up hierachical controller
  //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);
  _constraints_ptr.reset(
      new mwoibn::hierarchical_control::ConstraintsTask(_robot));
  mwoibn::Vector3 pelvis;
  pelvis << 0, 0, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                     robot.getLinks("base"));
  _pelvis_position_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSelectiveTask(pelvis_ph,
                                                               pelvis));
  pelvis << 1, 1, 1;
  _pelvis_orientation_ptr.reset(
      new mwoibn::hierarchical_control::OrientationSelectiveTask(
          mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                      robot.getLinks("base")),
          pelvis, _robot));

  mwoibn::VectorInt map = _robot.biMaps().get("full_body").get();

  _com_ptr.reset(new mwoibn::hierarchical_control::CenterOfMassTask2(_full_robot, map));
  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianFlatReferenceTask3(
          mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                   robot.getLinks("wheels")),
          _robot, _full_robot));

  mwoibn::Axis x, y, z, ax;
  z <<   0,  1,  0;
  y <<   0,  0, -1;
  x <<   1,  0,  0;
//  mwoibn::hierarchical_control::CastorAngle castor1(
  ax << 0, 1, 0;
  mwoibn::hierarchical_control::CastorAngle castor1(
      robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  1;
  mwoibn::hierarchical_control::CamberAngle camber1(
      robot, mwoibn::point_handling::Point("wheel_1", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  1;
  mwoibn::hierarchical_control::SteeringAngle steer1(
      robot, mwoibn::point_handling::Point("wheel_1", robot.getModel()), x, y,
      z, ax);
  ax << 0, 1, 0;
  mwoibn::hierarchical_control::CastorAngle castor3(
      robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  1;
  mwoibn::hierarchical_control::CamberAngle camber3(
      robot, mwoibn::point_handling::Point("wheel_3", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  1;
  mwoibn::hierarchical_control::SteeringAngle steer3(
      robot, mwoibn::point_handling::Point("wheel_3", robot.getModel()), x, y,
      z, ax);

  z <<  0, -1,  0;
  y <<  0,  0, -1;
  x << -1,  0,  0;

  ax << 0, -1, 0;
  mwoibn::hierarchical_control::CastorAngle castor2(
      robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  -1;
  mwoibn::hierarchical_control::CamberAngle camber2(
      robot, mwoibn::point_handling::Point("wheel_2", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  -1;
  mwoibn::hierarchical_control::SteeringAngle steer2(
      robot, mwoibn::point_handling::Point("wheel_2", robot.getModel()), x, y,
      z, ax);
  ax << 0, -1, 0;
  mwoibn::hierarchical_control::CastorAngle castor4(
      robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  -1;
  mwoibn::hierarchical_control::CamberAngle camber4(
      robot, mwoibn::point_handling::Point("wheel_4", robot.getModel()), x, y,
      z, ax);
  ax <<  0,  0,  -1;
  mwoibn::hierarchical_control::SteeringAngle steer4(
      robot, mwoibn::point_handling::Point("wheel_4", robot.getModel()), x, y,
      z, ax);

  _leg_steer_ptr.reset(new mwoibn::hierarchical_control::SteeringAngleTask(
      {steer1, steer2, steer3, steer4}, robot));
  _leg_camber_ptr.reset(new mwoibn::hierarchical_control::CamberAngleTask(
      {camber1, camber2, camber3, camber4}, robot));
  _leg_castor_ptr.reset(new mwoibn::hierarchical_control::CastorAngleTask(
      {castor1, castor2, castor3, castor4}, robot));

  int task = 0;
  int ratio = 1;
  double damp = 1e-4;
  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  _hierarchical_controller.addTask(_constraints_ptr.get(), gain, task, damp);
  task++;
  gain << 15 * ratio;
  _hierarchical_controller.addTask(_leg_steer_ptr.get(), gain, task, damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller.addTask(_pelvis_orientation_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller.addTask(_com_ptr.get(), gain, task, damp);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller.addTask(_steering_ptr.get(), gain, task, damp);
  task++;
  gain << 15 * ratio;
  _hierarchical_controller.addTask(_leg_camber_ptr.get(), gain, task, 0.04);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller.addTask(_leg_castor_ptr.get(), gain, task, 0.1);
  task++;
  gain << 25 * ratio;
  _hierarchical_controller.addTask(_pelvis_position_ptr.get(), gain, task,
                                   damp);
  task++;
  _dt = _robot.rate();

  _leg_steer_ptr->updateError();
  _leg_camber_ptr->updateError();
  _leg_castor_ptr->updateError();
  _steering_ptr->updateState();

  steerings = _leg_steer_ptr->getCurrent();

  mwoibn::VectorN init;
  init.setZero(4);
  _steering_ref_ptr.reset(new mgnss::events::Steering4(
      _robot, *_steering_ptr, init, 0.7, 0.3, _dt, 0.1));

  _leg_steer_ptr->setReference(steerings);
  _leg_camber_ptr->setReference(_leg_camber_ptr->getCurrent());
  _leg_castor_ptr->setReference(_leg_castor_ptr->getCurrent());

  _orientation = mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4])*mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]);

  _pelvis_orientation_ptr->setReference(0, _orientation);

  _position = _pelvis_position_ptr->points().getPointStateWorld(0);
  _position.head(2) = _full_robot.centerOfMass().get().head(2);
  _pelvis_position_ptr->setReference(_position);
  _com_ptr->setReference(_position.head(2));
  _heading = _steering_ptr->getState()[2];

  _linear_vel.setZero();
  _angular_vel.setZero();

  _select_steer = robot.getDof(robot.getLinks("camber"));
  _l_limits.setZero(_select_steer.size());
  _u_limits.setZero(_select_steer.size());
  robot.lower_limits.get(_l_limits, _select_steer);
  robot.upper_limits.get(_u_limits, _select_steer);

//  std::cout << "current state\t" << _steering_ptr->getState() << std::endl;
//  std::cout << "-PI" << _steering_ptr->getState()[4]-mwoibn::PI << std::endl;
//  std::cout << "current orientation\t" << _pelvis_orientation_ptr->points().getPointStateWorld(0) << std::endl;
//  std::cout << "orientation x\t" << mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]) << std::endl;
//  std::cout << "orientation y\t" << mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4]-mwoibn::PI) << std::endl;
//  std::cout << "orientation z\t" << mwoibn::Quaternion::fromAxisAngle(_z, _heading) << std::endl;
//  std::cout << "orientation\t" << _orientation << std::endl;
//  std::cout << "orientation xy\t" << mwoibn::Quaternion::fromAxisAngle(_z, _heading)*mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]) << std::endl;

//  std::cout << "my computation\t" << mwoibn::Quaternion::fromAxisAngle(_z, _heading)*_orientation << std::endl;

  _previous_command = mwoibn::VectorN::Zero(3);
  _command.setZero(_robot.getDofs());
}

void mwoibn::WheeledMotionEvent2::nextStep(const mwoibn::VectorN& support)
{

//  updateBase(velocity, omega);
  _full_robot.centerOfMass().update(true);

  updateSupport(support);
  updateBase();

  mwoibn::Vector3 com = _full_robot.centerOfMass().get();

  _next_step[0] =
      (_position[0] - com[0]) / _robot.rate();
  _next_step[1] =
      (_position[1] - com[1]) / _robot.rate();
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

void mwoibn::WheeledMotionEvent2::resetSteering()
{
  for (int i = 0; i < 4; i++)
  {
    _leg_steer_ptr->setReference(i, 0);
  }
}

double mwoibn::WheeledMotionEvent2::limit(const double th)
{
  return th - 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

void mwoibn::WheeledMotionEvent2::update(const mwoibn::VectorN& support)
{

  nextStep(support);
  compute();
}

void mwoibn::WheeledMotionEvent2::fullUpdate(const mwoibn::VectorN& support)
{
  _robot.get();
  _robot.updateKinematics();
  _full_robot.get();
  _full_robot.updateKinematics();

  update(support);

  _robot.send();
  _robot.wait();
}
void mwoibn::WheeledMotionEvent2::compute()
{
//_leg_castor_ptr->updateError();
//  _leg_steer_ptr->updateError();
  _command.noalias() = _hierarchical_controller.update();

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::VELOCITY);

  _command.noalias() = _command * _robot.rate();
  _command.noalias() +=
      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::POSITION);

  if (count == 30)
  {
//    std::cout << "com\t";
//    std::cout << _com_ptr->getError().transpose() << std::endl;
//    std::cout << "steer\t";
//    std::cout << steerings.transpose()*180/mwoibn::PI << std::endl;
//    std::cout << "error\t";
//    std::cout << _steering_ptr->getWorldError().transpose() << std::endl;

//    std::cout.precision(6);

//    std::cout << std::fixed << _leg_camber_ptr->getReference(0) * 180 / 3.14
//              << "\t";

//    std::cout << std::fixed << steerings.transpose() * 180 / 3.14 << "\t";
//    std::cout << std::fixed << _leg_steer_ptr->getCurrent()[0] * 180 / 3.14
//              << "\t";
//    std::cout << std::fixed << _leg_steer_ptr->getError().transpose() * 180 / 3.14
//              << std::endl;
//    std::cout << std::fixed << (_steering_ptr->getWorldError())[0] * 100
//              << "\t";
//    std::cout << std::fixed << (_steering_ptr->getWorldError())[1] * 100
//              << std::endl;
    count = 0;
  }
  else
    count++;
}


void mwoibn::WheeledMotionEvent2::steering()
{

  _steering_ref_ptr->compute(_next_step);

  steerings.noalias() = _steering_ref_ptr->get();

  for (int i = 0; i < 4; i++)
  {
    steerings[i] = (steerings[i] < _l_limits[i]) ? steerings[i] + mwoibn::PI : steerings[i];
    steerings[i] = (steerings[i] > _u_limits[i]) ? steerings[i] - mwoibn::PI : steerings[i];
    setSteering(i, steerings[i]);
//    std::cout << steerings.transpose()*180/mwoibn::PI << std::endl;
  }
}
