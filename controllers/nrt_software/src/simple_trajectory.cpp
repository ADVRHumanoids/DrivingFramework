
#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/visualization_tools/rviz_track_point.h>
#include <mwoibn/point_handling/robot_points_handler.h>
//#include <MathGeoLib/Math/float3.h>
//#include <MathGeoLib/Algorithm/Random/LCG.h>
//#include <MathGeoLib/Geometry/GeometryAll.h>
#include <mwoibn/hierarchical_control/cartesian_reference_task.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/self_collision_task.h>
#include <mwoibn/collision_model/robot_collision_ros.h>

#include <mwoibn/reference_generation/local_circle.h>
#include <mwoibn/reference_generation/line.h>

int main(int argc, char** argv)
{
  // start ros
  ros::init(argc, argv, "simple_trajectory"); // initalize node
  ros::NodeHandle node;
  ros::Rate rate(1000);
  ros::NodeHandle node_local("~");

  bool is_static = true;
/*  node_local.param("is_static", is_static,
                   false); // Node takes the parameter is_static to define if
                           // the robot has a floating base: call with arguemnt
                           // _is_static:=true if it doesn't
*/
  // init robot
  mwoibn::robot_class::RobotRosNRT robot("/robot_description", "", false, false);

  RigidBodyDynamics::Math::VectorNd command = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION); // in controller order
  RigidBodyDynamics::Math::VectorNd ref_states = command;

  // create rviz trackers
  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/cartesian_test");
  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE, "world", 0.002, 0.002,
                     0.002); // nr.0 - desired position
  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE, "world", 0.002, 0.002,
                     0.002); // nr.1 - desired position

  // prepare end-effectors controller
  Eigen::Vector3d P_0;
  P_0 << 0.0, 0.0, -0.18;

  // start cartesian position tracker
  mwoibn::point_handling::PositionsHandler ik_0("ROOT", robot, {"arm1_7", "arm2_7"}, {P_0, P_0}); // this is a PH for the end-effector, shouldn't be changed
  mwoibn::point_handling::PositionsHandler ik("pelvis", robot, {"arm1_7", "arm2_7"}, {P_0, P_0}); // this is a PH for the end-effector, shouldn't be changed

  mwoibn::hierarchical_control::CartesianReferenceTask cartesian(ik_0, "pelvis", robot);

  // prepare self-collision avoidance task
  mwoibn::collision_model::RobotCollisionRos mesh(
      robot, "/robot_description", "/robot_semantic_description", "centauro");
  std::vector<double> safety_limits(mesh.getPairsNumber(), 0.05);

  mwoibn::hierarchical_control::SelfCollisionTask self_collision(mesh, safety_limits);

  // set-up tasks stack
  mwoibn::hierarchical_control::HierarchicalController hierarchical_controller;
  // prepere and initialize tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1200;
  hierarchical_controller.addTask(&cartesian, gain, 0);
  gain << 800;
  hierarchical_controller.addTask(&self_collision, gain, 1);

  // Create trajectory
  Eigen::VectorXd x_0(6);
  Eigen::VectorXd x_des(6);

  ros::spinOnce();
  rate.sleep();
  Eigen::VectorXd x_init(6);

  x_init = ik.getFullStateReference(); // get current position of a robot's end-effectors
  Eigen::Vector3d P_1, P_2;
  P_1 << 0, 0, 0;
  P_2 << 0.7, 0.3, 0;
  mwoibn::point_handling::PositionsHandler ik_2("pelvis", robot, {"arm1_1", "arm1_1"}, {P_1, P_2});

  Eigen::Vector3d P_des_0, P_des_1;
  P_des_0 << 0.5, 0.2, 0.2;
  P_des_1 << 0.5, -0.2, 0.2;

//  mwoibn::point_handling::PositionsHandler ik_des("ROOT", &robot, {"pelvis", "pelvis"},  {P_des_0, P_des_1});
  Eigen::Vector3d org;
 // for (int i = 0; i < 10; i++)
 // {
    x_0 << ik.getFullStateReference(); // desired position in a current step
    x_des << 0.40, 0.2, 0.6, 0.40, -0.2, 0.6; // final position

    // init first reference
    Eigen::VectorXd ax = ik_2.getFullStateWorld();
    Eigen::Vector3d axis = ax.tail(3) - ax.head(3);
    mwoibn::reference_generation::Local_Circle circle_0(x_0.head(3), x_des.head(3), axis, 0.0002);

    P_1 << 0, 0, 0;
    P_2 << 0.7, -0.3, 0;
    mwoibn::point_handling::PositionsHandler ik_1("pelvis", robot, {"arm2_1", "arm2_1"}, {P_1, P_2}); // this is used to defined the cirle, and direction

    ax = ik_1.getFullStateReference();
    axis = ax.tail(3) - ax.head(3);

    Eigen::Vector3d P_track_0, P_track_1, com;
    P_track_0 << 0.5, 0.2, 0.2;
    P_track_1 << 0.5, 0.2, 0.2;
    mwoibn::point_handling::PositionsHandler cartesian_task_ph("ROOT", robot, {"pelvis","pelvis"}, {P_track_0, P_track_1}); // for trackers

    mwoibn::reference_generation::Local_Circle circle_1(x_0.tail(3), x_des.tail(3), axis, 0.0002);

    // perform circular trajectory program
    while ((x_0 - x_des).norm() > 0.001 & robot.isRunning())
    {
      // next step in circular trajectory
      x_0.head(3) = circle_0.nextStep();
      x_0.tail(3) = circle_1.nextStep();


      cartesian_task_ph.setPointStateFixed(0, x_0.head(3));
      cartesian_task_ph.setPointStateFixed(1, x_0.tail(3));

      // update markers
      tracker.updateMarker(0, cartesian_task_ph.getPointStateWorld(0));
      tracker.updateMarker(1, cartesian_task_ph.getPointStateWorld(1));
      tracker.publish();

      cartesian.setReference(x_0);

      command = hierarchical_controller.update() *
                    rate.expectedCycleTime().toSec() +
                robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
      if(!is_static)  command.head(6+4*6) << ref_states.head(6+4*6);
      robot.command.set(command,mwoibn::robot_class::INTERFACE::POSITION);
      robot.controllers.send();
      robot.update();
      }


    x_des << 1.0, 0.2, 0.6, 1.0, -0.2, 0.6; // 1.1, 0.2, 0.2, 1.1, -0.2, 0.2


    while ((x_0 - x_des).norm() > 0.00001 & robot.isRunning())
    {

      x_0.head(3) = mwoibn::reference_generation::makeLine(x_0.head(3), x_des.head(3), 0.0002);
      x_0.tail(3) = mwoibn::reference_generation::makeLine(x_0.tail(3), x_des.tail(3), 0.0002);

      // update markers
      cartesian_task_ph.setPointStateFixed(0, x_0.head(3));
      cartesian_task_ph.setPointStateFixed(1, x_0.tail(3));

      tracker.updateMarker(0, cartesian_task_ph.getPointStateWorld(0));
      tracker.updateMarker(1, cartesian_task_ph.getPointStateWorld(1));
      tracker.publish();

      cartesian.setReference(x_0);

      command = hierarchical_controller.update() *
                    rate.expectedCycleTime().toSec() +
                robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
      if(!is_static)  command.head(18) << ref_states.head(18);

      robot.command.set(command,mwoibn::robot_class::INTERFACE::POSITION);
      robot.controllers.send();
      robot.update();
    }


    P_1 << 0, 0, 0;
    P_2 << 0, 0, 0;
    mwoibn::point_handling::PositionsHandler ik_3("pelvis", robot, {"arm1_1", "arm2_1"}, {P_1, P_2});

    Eigen::VectorXd pos = ik_3.getFullStateReference();

    P_1(0) = 0.4;
    P_2(0) = 0.4;
    P_1(1) = x_0[1];
    P_2(1) = x_0[4];
    P_1(2) = 1.6; // 1.2
    P_2(2) = 1.6; // 1.2

    axis << 0.5, 0, 0.5;

    mwoibn::reference_generation::Local_Circle circle_2(x_0.head(3), P_1, pos.head(3), axis, 0.0005);
    axis << -0.5, 0, -0.5;

    mwoibn::reference_generation::Local_Circle circle_3(x_0.tail(3), P_2, pos.tail(3), axis, 0.0005);

    x_des.head(3) = circle_2.getFinalPoint();
    x_des.tail(3) = circle_3.getFinalPoint();

    while ((x_0 - x_des).norm() > 0.00001 & robot.isRunning())
    {

      x_0.head(3) = circle_2.nextStep();
      x_0.tail(3) = circle_3.nextStep();
      org = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(3);
      // update markers
      cartesian_task_ph.setPointStateFixed(0, x_0.head(3));
      cartesian_task_ph.setPointStateFixed(1, x_0.tail(3));

      tracker.updateMarker(0, cartesian_task_ph.getPointStateWorld(0));
      tracker.updateMarker(1, cartesian_task_ph.getPointStateWorld(1));
      tracker.publish();

      cartesian.setReference(x_0);

      command = hierarchical_controller.update() *
                    rate.expectedCycleTime().toSec() +
                robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
      if(!is_static)  command.head(18) << ref_states.head(18);

      robot.command.set(command,mwoibn::robot_class::INTERFACE::POSITION);
      robot.controllers.send();
      robot.update();
    }

    axis << 0, 0.5, -0.5;
    //  mwoibn::reference_generation::Local_Circle circle_4(x_0.head(3), P_1.row(0),  pos.head(3), axis,
    //  0.0005);
    mwoibn::reference_generation::Local_Circle circle_4(x_0.head(3), x_init.head(3), axis, 0.0002);

    axis << 0, -0.5, -0.5;
    //  mwoibn::reference_generation::Local_Circle circle_5(x_0.tail(3), P_1.row(1), pos.tail(3), axis,
    //  0.0005);
    mwoibn::reference_generation::Local_Circle circle_5(x_0.tail(3), x_init.tail(3), axis, 0.0002);

    x_des.head(3) = circle_4.getFinalPoint();
    x_des.tail(3) = circle_5.getFinalPoint();
    /*  LCG lcg(2000);
      for (int i = 0; i < 1000; i++){

        x_0.head(3) = circle_4.getPoint(lcg.Float(-3.15, 3.15));
        x_0.tail(3) = circle_5.getPoint(lcg.Float(-3.15, 3.15));


        tracker.updateMarker(0, x_0[0], x_0[1], x_0[2]);
        tracker.updateMarker(1, x_0[3], x_0[4], x_0[5]);
        tracker.publish();


        ros::spinOnce();

        rate.sleep();
      } */

    Eigen::VectorXd origin = circle_4.getOriginPoint();
    Eigen::VectorXd end = circle_4.getFinalPoint();

    origin = circle_5.getOriginPoint();
    end = circle_5.getFinalPoint();


    while ((x_0 - x_des).norm() > 0.00001 & robot.isRunning())
    {

      x_0.head(3) = circle_4.nextStep();
      x_0.tail(3) = circle_5.nextStep();
      //     std::cout << x_0 << std::endl;
      org = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(3);
      // update markers
      cartesian_task_ph.setPointStateFixed(0, x_0.head(3));
      cartesian_task_ph.setPointStateFixed(1, x_0.tail(3));

      tracker.updateMarker(0, cartesian_task_ph.getPointStateWorld(0));
      tracker.updateMarker(1, cartesian_task_ph.getPointStateWorld(1));
      tracker.publish();

      cartesian.setReference(x_0);

      command = hierarchical_controller.update() *
                    rate.expectedCycleTime().toSec() +
                robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

      if(!is_static)  command.head(18) << ref_states.head(18);

      robot.command.set(command,mwoibn::robot_class::INTERFACE::POSITION);
      robot.controllers.send();
      robot.update();
    }

  return 0;
}
