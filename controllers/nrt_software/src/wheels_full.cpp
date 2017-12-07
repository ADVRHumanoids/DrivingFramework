#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion_full.h>
#include <mgnss/controllers/wheeled_references.h>
#include <custom_services/updatePDGains.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
    custom_services::updatePDGains::Response& res,
    mwoibn::hierarchical_control::CartesianSelectiveTask* position,
    mwoibn::hierarchical_control::CastorAngleTask* castor, mwoibn::hierarchical_control::CamberAngleTask *camber, mwoibn::hierarchical_control::SteeringAngleTask* steer);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheels_reference"); // initalize node

  ros::NodeHandle n;

  // init wheels_controller
  mwoibn::loaders::Robot loader;

  mwoibn::robot_class::Robot& robot =
      loader.init("/home/malgorzata/catkin_ws/src/DrivingFramework/"
                  "locomotion_framework/configs/"
                  "mwoibn_v2.yaml",
                  "default");

  robot.wait();
  robot.get();
  robot.updateKinematics();

  mwoibn::Vector3 pelvis_s;
  pelvis_s << 1, 1, 1;

//  mwoibn::hierarchical_control::CartesianSelectiveTask pelvis(
//       mwoibn::point_handling::PositionsHandler("ROOT", robot,
//                                                    robot.getLinks("base")),
//        pelvis_s);

  mwoibn::VectorN selection(12);

//  selection << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;

//  mwoibn::hierarchical_control::OrientationSelectiveTask pelvis_orn(
//      mwoibn::point_handling::OrientationsHandler("ROOT", robot,
//                                                  robot.getLinks("base")),
//      pelvis_s, robot);

  selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

  mwoibn::hierarchical_control::CartesianSelectiveTask leg_position(
      mwoibn::point_handling::PositionsHandler("ROOT", robot,
                                               robot.getLinks("wheels")),
      selection);

  mwoibn::Axis x, y, z, ax;
  z << 0, 0, -1;
  y << 0, 1, 0;
  x << -1, 0, 0;
  ax << 0, 0, 1;

  mwoibn::hierarchical_control::CastorAngle castor1(
      robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
      z);
  mwoibn::hierarchical_control::CamberAngle camber1(
      robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
      z, ax);
  mwoibn::hierarchical_control::SteeringAngle steer1(
      robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
      z, ax);
  z << 0,  0, -1;
  y << 0, -1,  0;
  x << 1,  0,  0;
  mwoibn::hierarchical_control::CastorAngle castor2(
      robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
      z);
  mwoibn::hierarchical_control::CamberAngle camber2(
      robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
      z, ax);
  mwoibn::hierarchical_control::SteeringAngle steer2(
      robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
      z, ax);
  z << 0,  0, -1;
  y << 0, -1,  0;
  x << 1,  0,  0;
  mwoibn::hierarchical_control::CastorAngle castor3(
      robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
      z);
  mwoibn::hierarchical_control::CamberAngle camber3(
      robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
      z, ax);
  mwoibn::hierarchical_control::SteeringAngle steer3(
      robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
      z, ax);
  z <<  0,  0, -1;
  y <<  0,  1,  0;
  x << -1,  0,  0;
  mwoibn::hierarchical_control::CastorAngle castor4(
      robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
      z);
  mwoibn::hierarchical_control::CamberAngle camber4(
      robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
      z, ax);
  mwoibn::hierarchical_control::SteeringAngle steer4(
      robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
      z, ax);

  mwoibn::hierarchical_control::CastorAngleTask castor({castor1, castor2, castor3, castor4}, robot);
  mwoibn::hierarchical_control::CamberAngleTask camber({camber1, camber2, camber3, camber4}, robot);
  mwoibn::hierarchical_control::SteeringAngleTask steer({steer1, steer2, steer3, steer4}, robot);
  mwoibn::hierarchical_control::ConstraintsTask constraints(robot);

  mwoibn::VectorN command;

  int task = 0;
  int ratio = 1;
  double damp = 1e-4;
  // Set initaial HC tasks
  mwoibn::hierarchical_control::HierarchicalController hierarchical_controller;

  RigidBodyDynamics::Math::VectorNd gain(1);

  gain << 1;
//  hierarchical_controller.addTask(&constraints, gain, task, damp);
//  task++;
//  gain << 50 * ratio;
//  hierarchical_controller.addTask(&pelvis, gain, task, 1e-3);
//  task++;
//  gain << 50 * ratio;
//  hierarchical_controller.addTask(&pelvis_orn, gain, task, 1e-3);
//  task++;
  gain << 50 * ratio;
  hierarchical_controller.addTask(&steer, gain, task, 1e-3);
  task++;
//  gain << 25 * ratio;
//  hierarchical_controller.addTask(&castor, gain, task, 1e-3);
//  task++;
//  gain << 25 * ratio;
//  hierarchical_controller.addTask(&camber, gain, task, 0.04);
//  task++;




  command.setZero(robot.getDofs());

  // ros topics/service support
  ros::ServiceServer service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "wheels_command",
          boost::bind(&evenstHandler, _1, _2, &leg_position, &castor, &camber, &steer));


  castor.setReference(0, 0);
  castor.setReference(1, 0);
  castor.setReference(2, 0);
  castor.setReference(3, 0);
  camber.setReference(0, 0);
  camber.setReference(1, 0);
  camber.setReference(2, 0);
  camber.setReference(3, 0);
//  steer.setReference(0, 0);
//  steer.setReference(1, 0);
//  steer.setReference(2, 0);
//  steer.setReference(3, 0);
  leg_position.setReference(leg_position.points().getFullStateWorld());

  while (ros::ok())
  {
    command.noalias() = hierarchical_controller.update();

    robot.command.set(command, mwoibn::robot_class::INTERFACE::VELOCITY);

    command.noalias() = command * robot.rate();
    command.noalias() +=
        robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

    robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
    robot.update();
  }
}

bool evenstHandler(
    custom_services::updatePDGains::Request& req,
    custom_services::updatePDGains::Response& res,
    mwoibn::hierarchical_control::CartesianSelectiveTask* position,
    mwoibn::hierarchical_control::CastorAngleTask* castor,
    mwoibn::hierarchical_control::CamberAngleTask* camber,
    mwoibn::hierarchical_control::SteeringAngleTask* steer)
{
  if (req.p > 3 || req.p < 0)
  {
    res.message = "Unknown leg. Please chose number between 0 and 3";
    res.success = false;
    return true;
  }
  if (req.d > 5 || req.p < 0)
  {
    res.message = "Wrong reference\t x - 0 \t y - 1\t z - 2\t zeta - 3\t theta - 4\t psi - 5";
    res.success = false;
    return true;
  }
  mwoibn::Axis axis;

  if (req.d == 3)
  {
    castor->setReference(req.p, req.nr / 1.0 * 3.14 / 180);
    res.success = true;
    return true;
  }
  else if (req.d == 4)
  {
    camber->setReference(req.p, req.nr / 1.0 * 3.14 / 180);
    res.success = true;
    return true;
  }
  else if (req.d == 5)
  {
    steer->setReference(req.p, req.nr / 1.0 * 3.14 / 180);
    res.success = true;
    return true;
  }
  else if (req.d == 0)
    axis << 1, 0, 0;
  else if (req.d == 1)
    axis << 0, 1, 0;
  else if (req.d == 2)
    axis << 0, 0, 1;

  position->setReference(req.p,
                         position->getReference(req.p) + axis * req.nr / 100.0);
  res.success = true;
  return true;
}
