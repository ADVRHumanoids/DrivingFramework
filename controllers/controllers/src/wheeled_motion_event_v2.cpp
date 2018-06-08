#include <mgnss/controllers/wheeled_motion_event_v2.h>
#include <mgnss/controllers/steering_v4.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v6.h>

mgnss::controllers::WheeledMotionEvent2::WheeledMotionEvent2(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& full_robot)
    : WheelsControllerExtend(robot), _full_robot(full_robot)
{

  _robot.wait();
  _robot.get();
  _robot.updateKinematics();

  _full_robot.get();
  _full_robot.updateKinematics();

  _full_robot.centerOfMass().update(true);

  _createTasks();
  _initIK();

  mwoibn::VectorN init_steer;
  init_steer.setZero(4);
  _steering_ref_ptr.reset(new mgnss::events::Steering4(
      _robot, *_steering_ptr, init_steer, 0.7, 0.3, _robot.rate(), 0.1));

  _allocate();
  init();

}



void mgnss::controllers::WheeledMotionEvent2::_setInitialConditions(){
  WheelsControllerExtend::_setInitialConditions();

  _orientation = (mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4]) * mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]));

  _pelvis_orientation_ptr->setReference(0, _orientation);

  _position = _pelvis_position_ptr->points().getPointStateWorld(0);
  _position.head(2) = _full_robot.centerOfMass().get().head(2);
  _pelvis_position_ptr->setReference(_position);
  _com_ptr->setReference(_position.head(2));
  _heading = _steering_ptr->getState()[2];


}

void mgnss::controllers::WheeledMotionEvent2::_initIK(){

  int task = 0;
  int ratio = 1;
  double damp = 1e-4;
  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  _hierarchical_controller_ptr->addTask(_constraints_ptr.get(), gain, task, damp);
  task++;
  gain << 15 * ratio;
  _hierarchical_controller_ptr->addTask(_leg_steer_ptr.get(), gain, task, damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller_ptr->addTask(_pelvis_orientation_ptr.get(), gain, task,
                                   damp);
  task++;
  gain << 20 * ratio;
  _hierarchical_controller_ptr->addTask(_com_ptr.get(), gain, task, damp);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller_ptr->addTask(_steering_ptr.get(), gain, task, damp);
  task++;
  gain << 15 * ratio;
  _hierarchical_controller_ptr->addTask(_leg_camber_ptr.get(), gain, task, 0.04);
  task++;
  gain << 10 * ratio;
  _hierarchical_controller_ptr->addTask(_leg_castor_ptr.get(), gain, task, 0.1);
  task++;
  gain << 25 * ratio;
  _hierarchical_controller_ptr->addTask(_pelvis_position_ptr.get(), gain, task,
                                   damp);
  task++;
}

void mgnss::controllers::WheeledMotionEvent2::_createTasks(){

  _constraints_ptr.reset(
      new mwoibn::hierarchical_control::ConstraintsTask(_robot));
  mwoibn::Vector3 pelvis;
  pelvis << 0, 0, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                     _robot.getLinks("base"));
  _pelvis_position_ptr.reset(
      new mwoibn::hierarchical_control::CartesianSelectiveTask(pelvis_ph,
                                                               pelvis));
  pelvis << 1, 1, 1;
  _pelvis_orientation_ptr.reset(
      new mwoibn::hierarchical_control::OrientationSelectiveTask(
          mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                      _robot.getLinks("base")),
          pelvis, _robot));

  mwoibn::VectorInt map = _robot.biMaps().get("full_body").get();

  _com_ptr.reset(new mwoibn::hierarchical_control::CenterOfMassTask2(_full_robot, map));
  _steering_ptr.reset(
      new mwoibn::hierarchical_control::CartesianFlatReferenceTask3(
          mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                   _robot.getLinks("wheels")),
          _robot, _full_robot));

  _createAngleTasks();
}

void mgnss::controllers::WheeledMotionEvent2::fullUpdate(const mwoibn::VectorN& support)
{
  _robot.get();
  _robot.updateKinematics();
  _full_robot.get();
  _full_robot.updateKinematics();

  setSupport(support);
  update();

  _robot.send();
  _robot.wait();
}
