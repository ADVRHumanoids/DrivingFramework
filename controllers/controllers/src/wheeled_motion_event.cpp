#include <mgnss/controllers/wheeled_motion_event.h>

mwoibn::WheeledMotionEvent::WheeledMotionEvent(
    mwoibn::robot_class::Robot& robot)
    : _robot(robot)
{
  _x << 1, 0, 0;
  _y << 0, 1, 0;
  _z << 0, 0, 1;

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

  _com_ptr.reset(new mwoibn::hierarchical_control::CenterOfMassTask(_robot));

  _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianFlatReferenceTask2(
          mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                   robot.getLinks("wheels")),
          _robot, *_com_ptr.get()));

  mwoibn::Axis x, y, z, ax;
  z << 0, 1, 0;
  y << 0, 0, -1;
  x << 1, 0, 0;
  //  mwoibn::hierarchical_control::CastorAngle castor1(
  ax << 0, 1, 0;
  mwoibn::hierarchical_control::CastorAngle castor1(
      robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, 1;
  mwoibn::hierarchical_control::CamberAngle camber1(
      robot, mwoibn::point_handling::Point("wheel_1", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, 1;
  mwoibn::hierarchical_control::SteeringAngle steer1(
      robot, mwoibn::point_handling::Point("wheel_1", robot.getModel()), x, y,
      z, ax);
  ax << 0, 1, 0;
  mwoibn::hierarchical_control::CastorAngle castor3(
      robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, 1;
  mwoibn::hierarchical_control::CamberAngle camber3(
      robot, mwoibn::point_handling::Point("wheel_3", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, 1;
  mwoibn::hierarchical_control::SteeringAngle steer3(
      robot, mwoibn::point_handling::Point("wheel_3", robot.getModel()), x, y,
      z, ax);

  z << 0, -1, 0;
  y << 0, 0, -1;
  x << -1, 0, 0;

  ax << 0, -1, 0;
  mwoibn::hierarchical_control::CastorAngle castor2(
      robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, -1;
  mwoibn::hierarchical_control::CamberAngle camber2(
      robot, mwoibn::point_handling::Point("wheel_2", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, -1;
  mwoibn::hierarchical_control::SteeringAngle steer2(
      robot, mwoibn::point_handling::Point("wheel_2", robot.getModel()), x, y,
      z, ax);
  ax << 0, -1, 0;
  mwoibn::hierarchical_control::CastorAngle castor4(
      robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, -1;
  mwoibn::hierarchical_control::CamberAngle camber4(
      robot, mwoibn::point_handling::Point("wheel_4", robot.getModel()), x, y,
      z, ax);
  ax << 0, 0, -1;
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
  double ratio = 3.5; // 1.0 NRT, 3.5 RT

  double damp = 1e-4;
  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  _hierarchical_controller.addTask(_constraints_ptr.get(), gain, task, damp);
  task++;
  gain << 25 * ratio;

  _hierarchical_controller.addTask(_leg_steer_ptr.get(), gain, task, damp);
  task++;
  gain << 60 * ratio;
  _hierarchical_controller.addTask(_pelvis_orientation_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 45 * ratio;
  _hierarchical_controller.addTask(_com_ptr.get(), gain, task, damp);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller.addTask(_pelvis_position_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 22 * ratio; // 15
  _hierarchical_controller.addTask(_steering_ptr.get(), gain, task, damp);
  task++;
  gain << 15 * ratio; // 10
  _hierarchical_controller.addTask(_leg_camber_ptr.get(), gain, task, 0.04);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller.addTask(_leg_castor_ptr.get(), gain, task, 0.12);
  task++;

  _linear_vel.setZero();
  _angular_vel.setZero();

  _select_steer = robot.getDof(robot.getLinks("camber"));
  _select_wheel = robot.getDof(robot.getLinks("wheels"));
  _l_limits.setZero(_select_steer.size());
  _u_limits.setZero(_select_steer.size());
  _start_steer.setZero(_select_steer.size());
  _resteer.setConstant(_select_steer.size(), false);
  _current_steer.setZero(_select_steer.size());
  robot.lower_limits.get(_l_limits, _select_steer);
  robot.upper_limits.get(_u_limits, _select_steer);

  _test_steer.setZero(_select_steer.size());

  _com_ref.setZero(2);
  _steering_ref_ptr.reset(new mgnss::events::Steering5(
      _robot, *_steering_ptr, _test_steer, 0.8, 0.2, _robot.rate(), 0.03));

  _previous_command = mwoibn::VectorN::Zero(3);
  _command.setZero(_robot.getDofs());
}

void mwoibn::WheeledMotionEvent::init()
{
  _robot.wait();
  _robot.get();
  _robot.updateKinematics();
  _robot.centerOfMass().update();

  _steering_ptr->init();

  _dt = _robot.rate();

  _leg_steer_ptr->updateError();
  _leg_camber_ptr->updateError();
  _leg_castor_ptr->updateError();
  _steering_ptr->updateState();

  steerings = _leg_steer_ptr->getCurrent();

  _leg_steer_ptr->setReference(steerings);
  _leg_camber_ptr->setReference(_leg_camber_ptr->getCurrent());
  _leg_castor_ptr->setReference(_leg_castor_ptr->getCurrent());

  _orientation =
      mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4]) *
      mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]);

  _pelvis_orientation_ptr->setReference(0, _orientation);

  _position = _pelvis_position_ptr->points().getPointStateWorld(0);
  _position.head(2) = _robot.centerOfMass().get().head(2);
  _pelvis_position_ptr->setReference(_position);
  _com_ptr->setReference(_position.head(2));
  _heading = _steering_ptr->getState()[2];

  _previous_command = mwoibn::VectorN::Zero(3);
  _command.setZero(_robot.getDofs());
}

void mwoibn::WheeledMotionEvent::nextStep(const mwoibn::VectorN& support)
{

  //  updateBase(velocity, omega);
  _robot.centerOfMass().update();

  updateSupport(support);
  updateBase();
  mwoibn::Vector3 com = _robot.centerOfMass().get();

  _next_step[0] = (_position[0] - com[0]) / _robot.rate();
  _next_step[1] = (_position[1] - com[1]) / _robot.rate();
  _next_step[2] =
      (_heading - _steering_ptr->getState()[2]); // just limit the difference

  _next_step[2] -= 6.28318531 * std::floor((_next_step[2] + 3.14159265) /
                                           6.28318531); // limit -pi:pi
  _next_step[2] = _next_step[2] / _robot.rate();

  steering();
}

void mwoibn::WheeledMotionEvent::resetSteering()
{
  for (int i = 0; i < 4; i++)
  {
    _leg_steer_ptr->setReference(i, 0);
  }
}

double mwoibn::WheeledMotionEvent::limit(const double th)
{
  return th - 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

void mwoibn::WheeledMotionEvent::update(const mwoibn::VectorN& support)
{

  nextStep(support);
  compute();
}

void mwoibn::WheeledMotionEvent::fullUpdate(const mwoibn::VectorN& support)
{
  _robot.get();
  _robot.updateKinematics();

  update(support);

  _robot.send();
  _robot.wait();
}
void mwoibn::WheeledMotionEvent::compute()
{

  _command.noalias() = _hierarchical_controller.update();

  _steering_ref_ptr->resteer(_leg_steer_ptr->resteer());
  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::VELOCITY);

  _command.noalias() = _command * _robot.rate();

  _command.noalias() +=
      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::POSITION);

  _robot.state.get(_current_steer, _select_steer,
                   mwoibn::robot_class::INTERFACE::POSITION);
  _robot.command.get(_test_steer, _select_steer,
                     mwoibn::robot_class::INTERFACE::POSITION);

  std::cout.precision(6);

  // RESTEER AND CHECK FOR LIMITS
  for (int i = 0; i < _test_steer.size(); i++)
  {
    double steer = _test_steer[i];

    if ((_test_steer[i] < _l_limits[i] || _test_steer[i] > _u_limits[i]) && !_resteer[i])// && std::fabs(_test_steer[i] - _current_steer[i]) < 10*180/mwoibn::PI)
    {
      _start_steer[i] = _test_steer[i];
      _resteer[i] = true;
      std::cout << "WARNING: ankle yaw " << i << " on limit."
                << std::endl; // NRT
    }

//    std::cout << "\t" << i << "\t" << _resteer[i] << "\t" << _test_steer[i]*180/mwoibn::PI;

    if (_resteer[i])
    {
      eigen_utils::limitToHalfPi(_test_steer[i]);

      if (std::fabs(_test_steer[i]) > 1.0 &&
          std::fabs(_test_steer[i] - _start_steer[i]) < mwoibn::PI)
      {
        if (_start_steer[i] < 0)
          _test_steer[i] += mwoibn::HALF_PI;
        else
          _test_steer[i] -= mwoibn::HALF_PI;
      }

    }

    _resteer[i] =
        _resteer[i] &&
        (std::fabs(steer - _current_steer[i]) <
         (std::fabs(_test_steer[i] - _current_steer[i]) + 0.1)) &&
        std::fabs(_start_steer[i] - steer) < 2.5;

    if( _resteer[i] && _start_steer[i] < 0){
        _test_steer[i] = _current_steer[i] + 1*mwoibn::PI/180;
    }
    else if ( _resteer[i] &&  _start_steer[i] > 0){
        _test_steer[i] = _current_steer[i] - 1*mwoibn::PI/180;
    }

  }

//  std::cout << std::endl;
//  std::cout << steerings.transpose()*180/mwoibn::PI << std::endl;

//  std::cout << _resteer.transpose() << std::endl;
  _robot.command.set(_test_steer, _select_steer,
                     mwoibn::robot_class::INTERFACE::POSITION);


}

void mwoibn::WheeledMotionEvent::stop()
{
  _command.setZero();
  _robot.command.set(_command, mwoibn::robot_class::INTERFACE::VELOCITY);
  _robot.send();
}

void mwoibn::WheeledMotionEvent::steering()
{

  _steering_ref_ptr->compute2(_next_step);

  steerings.noalias() = _steering_ref_ptr->get();
  //  std::cout << steerings.transpose()*180/mwoibn::PI << std::endl;

  for (int i = 0; i < 4; i++)
  {
    setSteering(i, steerings[i]);
  }
  //  std::cout << steerings.transpose()*180/mwoibn::PI << std::endl;

  //  std::cout << "next step\t" << _next_step.transpose() << std::endl;
}
