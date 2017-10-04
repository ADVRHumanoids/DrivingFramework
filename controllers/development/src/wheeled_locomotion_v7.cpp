#include <mwoibn/robot_class/robot_ros_nrt.h>
//#include <mwoibn/robot_class/wheel_contact.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/controller_task.h>
#include <mwoibn/hierarchical_control/cartesian_reference_task.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v2.h>
#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/cartesian_world_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_world_task.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>

#include <mwoibn/point_handling/robot_points_handler.h>
//#endif
#include <custom_services/updatePDGains.h>
#include <custom_services/updatePrint.h>
#include <custom_messages/CustomCmnd.h>

// trajectory generation
#include <mwoibn/reference_generation/line.h>
#include <mwoibn/reference_generation/local_circle.h>
#include <programs/basic_handler.h>
#include <programs/steering.h>

// temp
#include <MathGeoLib/Algorithm/Random/LCG.h>
#include <MathGeoLib/Geometry/GeometryAll.h>

#include <mwoibn/eigen_utils/eigen_utils.h>

bool wheelHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  events::TrajectoryGenerator* events);

void trackerUpdater(
    mwoibn::visualization_tools::RvizTrackPoint& tracker,
    mwoibn::hierarchical_control::CartesianSelectiveTask& pelvis_position,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& leg_position,
    const double& z, mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis);

void nextStep(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::HierarchicalController&
        hierarchical_controller,
    mwoibn::visualization_tools::RvizTrackPoint& tracker,
    mwoibn::hierarchical_control::CartesianSelectiveTask& pelvis_position,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& leg_position,
    ros::Rate& rate, const double dt, const double& z, mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis);

int main(int argc, char** argv)
{

  static const mwoibn::visualization_tools::Utils::TYPE tracker_type =
      mwoibn::visualization_tools::Utils::TYPE::POINT;

  ros::init(argc, argv, "wheels_motion_2"); // initalize node

  ros::NodeHandle n;

  mwoibn::robot_class::RobotRosNRT robot("/home/user/malgorzata/workspace/src/locomotion_framework/configs/mwoibn_v2.yaml" ,"higher_scheme");

  // init current set-up
  ros::Rate rate(100);
  robot.update();

  // just to be sure, set controller initial state
  robot.command.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                    mwoibn::robot_class::INTERFACE::POSITION);

  // Set-up hierachical controller
  //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);
  mwoibn::hierarchical_control::ConstraintsTask constraints_task(robot);

  // decalare all variables
  int task = 0;
  double orientation = -2 * 3.1416, dt = rate.expectedCycleTime().toSec(),
         eps = 1.0 * 1e-3, ds = 0.005, dss = 0.005, t = 0, pi2 = 1.57079632679,
         r = 0.15, z = 0;
  mwoibn::Vector3 axis, pelvis;
  mwoibn::VectorN selection(12), ref(2), steerings(4), steering_error,
      leg_tracking, next_position(2), initial_state, current_position,
      next_step;

  //  std::vector<int> scale = {1, 1, 1, -1, -1, 1, -1, -1};
  mwoibn::VectorN scale(8);
  scale << 1, 1, 1, -1, -1, 1, -1, -1;
  mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask leg_position(
      mwoibn::point_handling::PositionsHandler(
          "ROOT", robot,
          std::vector<std::string>{"wheel_1", "wheel_2", "wheel_3", "wheel_4"}),
      robot);

  selection << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;

  mwoibn::hierarchical_control::OrientationSelectiveTask leg_orientation_z(
      mwoibn::point_handling::OrientationsHandler(
          "ROOT", robot, std::vector<std::string>{"ankle2_1", "ankle2_2",
                                                  "ankle2_3", "ankle2_4"}),
      selection, robot);
  selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

  mwoibn::hierarchical_control::OrientationSelectiveTask leg_orientation_xy(
      mwoibn::point_handling::OrientationsHandler(
          "ROOT", robot, std::vector<std::string>{"ankle2_1", "ankle2_2",
                                                  "ankle2_3", "ankle2_4"}),
      selection, robot);

  pelvis << 1, 1, 1;
  mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", robot, {"pelvis"});
  mwoibn::hierarchical_control::CartesianSelectiveTask pelvis_position(
      pelvis_ph, pelvis);

  pelvis << 1, 1, 1;
  mwoibn::hierarchical_control::OrientationSelectiveTask pelvis_orientation(
      mwoibn::point_handling::OrientationsHandler("ROOT", robot, {"pelvis"}),
      pelvis, robot);

  mwoibn::hierarchical_control::HierarchicalController hierarchical_controller;

  // Set initaial HC tasks
  RigidBodyDynamics::Math::VectorNd gain(1);
  gain << 1;
  hierarchical_controller.addTask(&constraints_task, gain, task, 1e-5);
  task++;
  gain << 120;
  hierarchical_controller.addTask(&leg_orientation_z, gain, task, 5 * 1e-3);
  task++;
  gain << 70;
  hierarchical_controller.addTask(&pelvis_position, gain, task, 5 * 1e-3);
  task++;
  gain << 70;
  hierarchical_controller.addTask(&pelvis_orientation, gain, task, 5 * 1e-3);
  task++;
  gain << 10;
  hierarchical_controller.addTask(&leg_orientation_xy, gain, task, 0.01);
  task++;
  gain << 15;
  hierarchical_controller.addTask(&leg_position, gain, task, 5 * 1e-3);
  task++;

  hierarchical_controller.update();

  // create trackers classes
  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");

  for (int i = 0; i < 11; i++)
    tracker.initMarker(tracker_type, "world", 0.002, 0.002, 0.002);

  // init controller and tracker
  robot.update();

  // define references

  // support polygon
  for (int i = 0; i < leg_position.points().size(); i++)
  {
    ref << scale[2 * i] * 0.40, scale[2 * i + 1] * 0.22;
    leg_position.setReference(i, ref);
  }

  // pelvis position
  ref = pelvis_position.getReference(0);
  ref(2) = 0.42;
  pelvis_position.setReference(ref);

  // pelvis orientation -- set it out of zero due to the representation
  // singularity
  axis << 0, 0, 1;
  pelvis_orientation.setReference(
      0, pelvis_orientation.getOffset(0) *
             mwoibn::Quaternion::fromAxisAngle(axis, orientation));

  hierarchical_controller.update();
  mwoibn::VectorN log_pelvis(3);
  mwoibn::VectorN log_position(8);
  // this loop is to make sure the controller is statically stable
  while (ros::ok() &&
         (leg_orientation_z.getError().cwiseAbs().maxCoeff() >
              100 * eps * 3.1416 / 180 ||
          pelvis_orientation.getError().cwiseAbs().maxCoeff() >
              100 * eps * 3.1416 / 180 ||
          pelvis_position.getError().cwiseAbs().maxCoeff() > 10 * eps))
  {
    //    std::cout << "part 1.1\t"
    //              << robot.state.get(mwoibn::robot_class::INTERFACE::POSITION)
    //                     .head(6) << std::endl;
    //    std::cout << "part 1.2\t"
    //              << leg_orientation_z.getError().cwiseAbs().maxCoeff()
    //              << std::endl;
    //    std::cout << "part 1.3\t"
    //              << pelvis_orientation.getError().cwiseAbs().maxCoeff()
    //              << std::endl;
    //    std::cout << "part 1.4\t" << leg_position.getError() << std::endl;
    for (int i = 0; i < 4; i++)
      log_position.segment<2>(2 * i) = leg_position.getReferenceWorld(i, true).head(2);

    log_pelvis = pelvis_position.getReference();

    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
             leg_position, rate, dt, z, log_position, log_pelvis);
  }

  //    // now check just support polygon tracking

  initial_state = leg_position.getReference();
  next_step.resize(3);
  next_step << 0, 0, 0;
  int path = 0;
  next_position = initial_state;
  mwoibn::VectorN min(8), max = initial_state + scale * 0.2;
  min << 0.30, 0.00, 0.30, 0.00, 0.30, 0.00, 0.30, 0.00;

  min = initial_state - scale.cwiseProduct(min);
  bool done = true;
  ds = ds / 4;

  while (robot.isRunning() && path < 6)
  {
    //    t += ds;
    for (int i = 0; i < 4; i++)
      log_position.segment<2>(2 * i) = leg_position.getReferenceWorld(i, true).head(2);

    log_pelvis = pelvis_position.getReference();

    // reference generation
    // move forward
    switch (path)
    {
    case 0:
      done = true;
      for (int i = 0; i < 4; i++)
      {
        next_position[2 * i + 0] += scale[2 * i + 0] * 4 * ds;
        if (std::fabs(next_position[2 * i + 0]) < std::fabs(max[2 * i + 0]))
          done = false;
      }
      if (done)
        path++;
      break;
    // wait until it will converge
    case 1:
      if (leg_position.getError().cwiseAbs().maxCoeff() < 50 * eps)
      {
        path++;
      }
      break;
    // get back
    case 2:
      done = true;
      for (int i = 0; i < 4; i++)
      {
        if (std::fabs(next_position[2 * i + 0]) > std::fabs(min[2 * i + 0]))
        {
          next_position[2 * i + 0] -= scale[2 * i + 0] * 4 * ds;
          done = false;
        }
      }
      if (pelvis_position.getReference()[2] < 0.50)
      {
        ref = pelvis_position.getReference(0);
        ref(2) += ds / 2;
        pelvis_position.setReference(ref);
        done = false;
      }
      if (done)
        path++;
      break;
    // wait until it will converge
    case 3:
      if (leg_position.getError().cwiseAbs().maxCoeff() < 100 * eps)
      {
        initial_state = leg_position.getReference();
        t = 0;
        path++;
      }
      break;
    //    case 4:
    //      done = true;
    //      if (pelvis_position.getReference()[2] > 0.30)
    //      {
    //        ref = pelvis_position.getReference(0);
    //        ref(2) -= ds;
    //        pelvis_position.setReference(ref);
    //        done = false;
    //      }
    //      else
    //        path++;
    //      break;
    //    case 5:
    //      if (pelvis_position.getError()[2] < 50 * eps)
    //      {
    //        path++;
    //      }
    //    case 6:
    ////      t += ds;
    //      done = true;
    //      for (int i = 0; i < 4; i++)
    //      {
    //        if (std::fabs(next_position[2 * i + 0]) > (std::fabs(min[2 * i +
    //        0])))
    //        {
    //          next_position[2 * i + 0] -= scale[2 * i + 0] * ds / 4;
    //          next_position[2 * i + 1] += scale[2 * i + 1] * ds / 2;
    //          done = false;
    //        }
    //        std::cout << "next_position\t max: " << i <<  std::endl;
    //        std::cout << next_position[2 * i + 0] << "\t";
    //        std::cout << max[2 * i + 0] << std::endl;

    //      }

    //      if (pelvis_position.getReference()[2] > 0.42)
    //      {
    //        ref = pelvis_position.getReference(0);
    //        ref(2) -= ds / 2;
    //        pelvis_position.setReference(ref);
    ////        done = false;
    //      }
    //      if (done)
    //        path++;

    //      break;
    //    case 7:
    //      if (leg_position.getError().cwiseAbs().maxCoeff() < 100 * eps)
    //        path++;
    //      break;
    case 4:
      //      t += ds;
      done = true;
      for (int i = 0; i < 4; i++)
      {
        if (std::fabs(next_position[2 * i + 1]) <
            (std::fabs(min[2 * i + 1]) + 0.3))
        {
          //          next_position[2 * i + 0] += scale[2 * i + 0] * ds / 4;
          next_position[2 * i + 1] += scale[2 * i + 1] * ds / 2;
          done = false;
        }
        if (std::fabs(next_position[2 * i + 0]) <
            (std::fabs(min[2 * i + 0]) + 0.07))
        {
          next_position[2 * i + 0] += scale[2 * i + 0] * ds / 4;
          done = false;
        }
        //        std::cout << "next_position\t max: " << i <<  std::endl;
        //        std::cout << next_position[2 * i + 1] << "\t";
        //        std::cout << max[2 * i + 1] << std::endl;
      }

      //      if (pelvis_position.getReference(0)[2] > 0.35)
      //      {
      //        ref = pelvis_position.getReference(0);
      //        ref(2) -= ds / 2;
      //        pelvis_position.setReference(ref);
      //        done = false;
      //      }
      if (done)
      {
        hierarchical_controller.updateDamping(1, 1e-8);
        hierarchical_controller.updateDamping(1, 1e-8);
        hierarchical_controller.updateDamping(2, 1e-8);
        hierarchical_controller.updateDamping(3, 1e-8);
        hierarchical_controller.updateDamping(4, 0.01);
        hierarchical_controller.updateDamping(5, 1e-8);
        hierarchical_controller.update();
        path++;
      }

      break;
    case 5:
      if (leg_position.getError().cwiseAbs().maxCoeff() < 100 * eps)
        path++;
      break;
    }

    for (int i = 0; i < 4; i++)
    {
      leg_position.setReference(i, next_position.segment<2>(2 * i));
    }

    events::combined2(robot, leg_position, steerings, next_step, 0.3, 0.7, dt);

    for (int i = 0; i < 4; i++)
    {
      leg_orientation_z.setReference(
          i, leg_orientation_z.getOffset(i) *
                 mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
    }

    // logging errors
    //    leg_tracking = leg_position.getError();
    //    steering_error = leg_orientation_z.getError();

    //    std::cout << "part 2.1\t" << path << std::endl;
    //    std::cout << "part 2.2\t" << next_position << std::endl;
    //    std::cout << "part 2.3\t" << leg_position.getError() << std::endl;

    //        std::cout << "part 3.1\t" << steering_error[2] * 180 / 3.14 <<
    //"\n";
    //    std::cout << "\t" << steering_error[5] * 180 / 3.14 << "\n";
    //    std::cout << "\t" << steering_error[8] * 180 / 3.14 << "\n";
    //    std::cout << "\t" << steering_error[11] * 180 / 3.14 << std::endl;
    //    std::cout << "part 3.2\t" << steerings * 180 / 3.14 << std::endl;

    //    std::cout << "part 3.3\t" << leg_tracking << std::endl;

    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
             leg_position, rate, dt, z, log_position, log_pelvis);
  }

  double orientation_ref = 0;
  next_position.resize(3);
  initial_state = pelvis_position.getReference();

  orientation = 0;

  r = 2;
  initial_state = pelvis_position.getReference();
  //  t = 0.005;
  t = 0;
  next_position.resize(3);
  next_position << r* std::cos(t - pi2) + initial_state[0],
      r * std::sin(t - pi2) + r + initial_state[1], // orientation;
      std::atan2(std::sin(t), std::cos(t)) + orientation;

  steering_error = mwoibn::VectorN::Ones(12);

  ref = pelvis_position.getReference(0);
  ref.head(2) = next_position.head(2);
  pelvis_position.setReference(0, ref);

  pelvis_orientation.setReference(
      0, pelvis_orientation.getOffset(0) *
             mwoibn::Quaternion::fromAxisAngle(axis, next_position[2]));

  hierarchical_controller.update();

  t = 0;
  ds = 0.001;
  path = 0;
  while (robot.isRunning())
  {
    leg_tracking = leg_position.getError();
    steering_error = leg_orientation_z.getError();
    for (int i = 0; i < 4; i++)
      log_position.segment<2>(2 * i) = leg_position.getReferenceWorld(i, true).head(2);

    log_pelvis = pelvis_position.getReference();

    next_position << r* std::cos(t - pi2) + initial_state[0],
        r * std::sin(t - pi2) + r + initial_state[1], // orientation;
        std::atan2(std::sin(t), std::cos(t)) + orientation;

    current_position = leg_position.getState().head(3);
    next_step = (next_position - current_position);

    next_step[2] -=
        6.28318531 * std::floor((next_step[2] + 3.14159265) / 6.28318531);
    // normalize to -pi:pi

    next_step = next_step / dt;

    leg_tracking = leg_position.getError();

    //    events::combined(robot, leg_position, steerings, next_step, 0.7, 0.3,
    //                     2 * 1e-3);

    events::combined2(robot, leg_position, steerings, next_step, 0.3, 0.7, dt);

    for (int i = 0; i < 4; i++)
    {

      leg_orientation_z.setReference(
          i, leg_orientation_z.getOffset(i) *
                 mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
    }

    ref.head(2) = next_position.head(2);
    pelvis_position.setReference(0, ref);

    pelvis_orientation.setReference(
        0, pelvis_orientation.getOffset(0) *
               mwoibn::Quaternion::fromAxisAngle(axis, next_position[2]));

    t += ds;

    if (leg_tracking.cwiseAbs().maxCoeff() < 0.05 && path > 500 && ds < 0.005)
    {
      ds += 0.001;
      path = 0;
    }
    else
      path++;
    //      std::cout << "part 5.1\t" << ds << std::endl;
    //      std::cout << "part 5.2\t" << leg_tracking.segment<2>(0).norm() <<
    //      "\n";
    //      std::cout << "\t" << leg_tracking.segment<2>(2).norm() << "\n";
    //      std::cout << "\t" << leg_tracking.segment<2>(4).norm() << "\n";
    //      std::cout << "\t" << leg_tracking.segment<2>(6).norm() << std::endl;
    //      std::cout << "part 5.3\t" << steering_error[2] * 180 / 3.14 << "\n";
    //      std::cout << "\t" << steering_error[5] * 180 / 3.14 << "\n";
    //      std::cout << "\t" << steering_error[8] * 180 / 3.14 << "\n";
    //      std::cout << "\t" << steering_error[11] * 180 / 3.14 << std::endl;
    //      std::cout << "part 5.4\t" << pelvis_position.getError().norm() <<
    //      "\n";

    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
             leg_position, rate, dt, z, log_position, log_pelvis);
  }
}

bool wheelHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  events::TrajectoryGenerator* events)
{
  if (req.nr == 1)
  {
    res.success = events->line(req.p, req.d);
  }
  else if (req.nr == 0)
  {
    res.success = events->stop();
  }
  else if (req.nr == 2)
  {
    res.success = events->circle(req.p, req.d);
  }
  else
  {
    res.message = "Unknown command. Avaiable options:\n 0:\t stop\n 1:\t line "
                  "\n 2:\t circle";
    res.success = false;
  }
  return true;
}


void trackerUpdater(
    mwoibn::visualization_tools::RvizTrackPoint& tracker,
    mwoibn::hierarchical_control::CartesianSelectiveTask& pelvis_position,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& leg_position,
    const double& z, mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis)
{

  mwoibn::Vector3 reference;

  tracker.updateMarker(0, log_pelvis);
  tracker.updateMarker(1, pelvis_position.points().getPointStateWorld(0));

  tracker.updateMarker(2, leg_position.points().getPointStateWorld(0));
  reference.head(2) = log_position.segment<2>(0);
  reference[2] = z;
  tracker.updateMarker(3, reference);

  tracker.updateMarker(4, leg_position.points().getPointStateWorld(1));
  reference.head(2) = log_position.segment<2>(2);
  reference[2] = z;
  tracker.updateMarker(5, reference);

  tracker.updateMarker(6, leg_position.points().getPointStateWorld(2));
  reference.head(2) = log_position.segment<2>(4);
  reference[2] = z;
  tracker.updateMarker(7, reference);

  tracker.updateMarker(8, leg_position.points().getPointStateWorld(3));
  reference.head(2) = log_position.segment<2>(6);
  reference[2] = z;
  tracker.updateMarker(9, reference);

  tracker.publish();
}

void nextStep(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::HierarchicalController&
        hierarchical_controller,
    mwoibn::visualization_tools::RvizTrackPoint& tracker,
    mwoibn::hierarchical_control::CartesianSelectiveTask& pelvis_position,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& leg_position,
    ros::Rate& rate, const double dt, const double& z,
    mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis)
{

  mwoibn::VectorN command = hierarchical_controller.update();

//  for (int i = 0; i < command.size(); i++)
//    std::cout << command[i] << "\t";
//  std::cout << std::endl;

  command = command * rate.expectedCycleTime().toSec() +
            robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);

  robot.controllers.send();

  trackerUpdater(tracker, pelvis_position, leg_position, z, log_position,
                 log_pelvis);

  robot.update();
}
