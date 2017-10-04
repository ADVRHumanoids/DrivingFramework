#include <mwoibn/robot_class/robot_ros_nrt.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/controller_task.h>
#include <mwoibn/hierarchical_control/cartesian_reference_task.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v3.h>
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
#include <mgnss/communication/basic_handler.h>
#include <mgnss/controllers/steering.h>

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
    const double& z);

void nextStep(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::HierarchicalController&
        hierarchical_controller,
    mwoibn::visualization_tools::RvizTrackPoint& tracker,
    mwoibn::hierarchical_control::CartesianSelectiveTask& pelvis_position,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& leg_position,
    double rate, const double dt, const double& z,
    mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis);

int main(int argc, char** argv)
{

  static const mwoibn::visualization_tools::Utils::TYPE tracker_type =
      mwoibn::visualization_tools::Utils::TYPE::LINE;

  ros::init(argc, argv, "wheels_motion_2"); // initalize node

  ros::NodeHandle n;

  mwoibn::robot_class::RobotRosNRT robot("/home/user/malgorzata/workspace/src/locomotion_framework/configs/mwoibn_v2.yaml" ,"higher_scheme");

  double rate = 200;

  // init current set-up
  robot.update();

  // just to be sure, set controller initial state
  robot.command.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                    mwoibn::robot_class::INTERFACE::POSITION);

  // Set-up hierachical controller
  //  mwoibn::hierarchical_control::CenterOfMassTask com_task(robot);
  mwoibn::hierarchical_control::ConstraintsTask constraints_task(robot);

  // decalare all variables
  int task = 0;
  double orientation = -2 * 3.1416, dt = 1/rate,
         eps = 1.0 * 1e-3, ds = 0.005, dss = 0.002, t = 0, pi2 = 1.57079632679,
         r = 0.35, z = 0;
  mwoibn::Vector3 axis, pelvis;
  mwoibn::VectorN selection(12), ref(2), steerings(4), steering_error,
      leg_tracking, next_position(2), initial_state(8), current_position,
      next_step;

  std::vector<int> scale = {1, 1, 1, -1, -1, 1, -1, -1};

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
  gain << 300;
  hierarchical_controller.addTask(&leg_orientation_z, gain, task, 1e-5);
  task++;
  gain << 200;
  hierarchical_controller.addTask(&pelvis_position, gain, task, 1e-5);
  task++;
  gain << 200;
  hierarchical_controller.addTask(&pelvis_orientation, gain, task, 1e-5);
  task++;
  gain << 120;
  hierarchical_controller.addTask(&leg_orientation_xy, gain, task, 0.01);
  task++;
  gain << 60;
  hierarchical_controller.addTask(&leg_position, gain, task, 1e-5);
  task++;

//  leg_position.setHight(0.0*Eigen::VectorXd::Ones(4));
  hierarchical_controller.update();

  // create trackers classes
  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");

  for (int i = 0; i < 11; i++)
    tracker.initMarker(tracker_type, "world", 0.002, 0.002, 0.002);

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

  log_pelvis = pelvis_position.getReference();
  // this loop is to make sure the controller is statically stable
  while (robot.isRunning() &&
         (leg_orientation_z.getError().cwiseAbs().maxCoeff() >
              100 * eps * 3.1416 / 180 ||
          pelvis_orientation.getError().cwiseAbs().maxCoeff() >
              100 * eps * 3.1416 / 180 ||
          pelvis_position.getError().cwiseAbs().maxCoeff() > 10 * eps))
  {
    for (int i = 0; i < 4; i++)
      log_position.segment<2>(2 * i) =
          leg_position.getReferenceWorld(i, true).head(2);

    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
             leg_position, rate, dt, z, log_position, log_pelvis);
  }

  for (int i = 0; i < leg_position.points().size(); i++)
  {
    initial_state.segment<2>(2 * i) << scale[2 * i] * 0.225,
        scale[2 * i + 1] * 0.125;
  }

  next_step.resize(3);
  next_step << 0, 0, 0;

  double t_max =   -0.3 * 3.1416 / 180;
  double t_min = -80 * 3.1416 / 180;

//  while (robot.isRunning() && z < 0.025)
//  {
//    t += ds;

//    // reference generation
//    if (t > t_max || t < t_min)
//    {
//      if (leg_position.getError().head(8).cwiseAbs().maxCoeff() > 0.01)
//        ds = 0;
//      else
//      {
//        ds = (t > (t_max+t_min)/2) ? -dss : dss;
//        z += 0.005;
//      }
//      std::cout << "t\t" << t << std::endl;
//    }

//    for (int i = 0; i < 4; i++)
//      log_position.segment<2>(2 * i) =
//          leg_position.getReferenceWorld(i, true).head(2);

//    log_pelvis = pelvis_position.getReference();
//    // step reference
//    for (int i = 0; i < 4; i++)
//    {

//      next_position << scale[2 * i] * r* std::cos(t) + initial_state[i * 2 + 0],
//          -scale[2 * i + 1] * r * std::sin(t) + initial_state[i * 2 + 1];

//      leg_position.setReference(i, next_position);
//    }
//    //        steerings[i] = events::PT(leg_position, steerings[i], i, 10 *
//    //        eps);
//    //          events::jointLimits(steerings[i]);
//    events::combined2(robot, leg_position, steerings, next_step, 0.3, 0.7, dt);

//    for (int i = 0; i < 4; i++)
//    {
//      leg_orientation_z.setReference(
//          i, leg_orientation_z.getOffset(i) *
//                 mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
//    }

//    // logging errors
//    //    leg_tracking = leg_position.getError();
//    //    steering_error = leg_orientation_z.getError();

//    //    std::cout << "part 3.1\t" << steering_error[2] * 180 / 3.14 <<
//    //"\n";
//    //    std::cout << "\t" << steering_error[5] * 180 / 3.14 << "\n";
//    //    std::cout << "\t" << steering_error[8] * 180 / 3.14 << "\n";
//    //    std::cout << "\t" << steering_error[11] * 180 / 3.14 << std::endl;
//    //    std::cout << "part 3.2\t" << steerings * 180 / 3.14 << std::endl;

//    //    std::cout << "part 3.3\t" << leg_tracking << std::endl;

//    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
//             leg_position, rate, dt, z, log_position, log_pelvis);
//  }

  // reset support polygon
  ref.resize(2);
  for (int i = 0; i < leg_position.points().size(); i++)
  {
    ref << scale[2 * i] * 0.40, scale[2 * i + 1] * 0.22;
    leg_position.setReference(i, ref);
  }

  hierarchical_controller.update();

  double orientation_ref = 0;
  next_position.resize(3);
  initial_state = pelvis_position.getReference();

//  while (robot.isRunning() &&
//         (leg_position.getError().cwiseAbs().maxCoeff() > 25 * eps ||
//          pelvis_orientation.getError().cwiseAbs().maxCoeff() >
//              100 * eps * 3.18 / 180 ||
//          std::fabs(orientation_ref) < std::fabs(orientation)))

//  {
//    leg_tracking = leg_position.getError();
//    steering_error = leg_orientation_z.getError();
//    for (int i = 0; i < 4; i++)
//      log_position.segment<2>(2 * i) =
//          leg_position.getReferenceWorld(i, true).head(2);
//    log_pelvis = pelvis_position.getReference();

//    if (std::fabs(orientation_ref) < std::fabs(orientation))
//      orientation_ref += (orientation > 0) ? 0.005 : -0.005;
//    else
//      orientation_ref = orientation;

//    next_position << initial_state[0], initial_state[1], orientation_ref;

//    current_position = leg_position.getState().head(3);

//    next_step = (next_position - current_position);

//    next_step[2] -= 6.28318531 * std::floor((next_step[2] + 3.14159265) /
//                                            6.28318531); // normalize to -pi:pi

//    next_step = next_step / dt;
//    //    events::combined(robot, leg_position, steerings, next_step, 1.0, 0.0,
//    //                     0.5 * 1e-2);
//    events::combined2(robot, leg_position, steerings, next_step, 0.3, 0.7, dt);

//    for (int i = 0; i < 4; i++)
//    {
//      leg_orientation_z.setReference(
//          i, leg_orientation_z.getOffset(i) *
//                 mwoibn::Quaternion::fromAxisAngle(axis, steerings[i]));
//    }

//    pelvis_orientation.setReference(
//        0, pelvis_orientation.getOffset(0) *
//               mwoibn::Quaternion::fromAxisAngle(axis, orientation_ref));

//    leg_tracking = leg_position.getError();

//    //        std::cout << "part 3.1\t" << steering_error[2] * 180 / 3.14 <<
//    //        "\n";
//    //        std::cout << "\t" << steering_error[5] * 180 / 3.14 << "\n";
//    //        std::cout << "\t" << steering_error[8] * 180 / 3.14 << "\n";
//    //        std::cout << "\t" << steering_error[11] * 180 / 3.14 << std::endl;
//    //        std::cout << "part 3.2\t" << steerings * 180 / 3.14 << std::endl;
//    //        std::cout << "part 3.3\t" << leg_tracking.segment<2>(0).norm() <<
//    //        "\n";
//    //        std::cout << "\t" << leg_tracking.segment<2>(2).norm() << "\n";
//    //        std::cout << "\t" << leg_tracking.segment<2>(4).norm() << "\n";
//    //        std::cout << "\t" << leg_tracking.segment<2>(6).norm() <<
//    //        std::endl;
//    nextStep(robot, hierarchical_controller, tracker, pelvis_position,
//             leg_position, rate, dt, z, log_position, log_pelvis);
//  }

  orientation -=
      6.28318531 * std::floor((orientation + 3.14159265) / 6.28318531);

  r = 2;
  initial_state = pelvis_position.getReference();
  t = 0.000;
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
  for (int i = 0; i < 4; i++)
  {
    leg_orientation_z.setReference(
        i, leg_orientation_z.getOffset(i) *
               mwoibn::Quaternion::fromAxisAngle(axis, 0));
  }
  leg_orientation_z.updateError();

    while (robot.isRunning() &&
           (steering_error.cwiseAbs().maxCoeff() > 500 * eps * 3.1416 / 180))
    {

      steering_error = leg_orientation_z.getError();

      nextStep(robot, hierarchical_controller, tracker, pelvis_position,
               leg_position, rate, dt, z, log_position, log_pelvis);
    }

  int path = 0, wait = 0, mode = 0;
  double ds_max = 0.5/rate, ds_step;
  ds = ds_max/10;
  ds_step = ds_max/10;
  double dsp = dss, r_sp = 0.35, st = -10*3.1416/180, sp = st;
  mwoibn::VectorN leg_sp(2), leg_init(8);

  for (int i = 0; i < leg_position.points().size(); i++)
  {
    leg_init.segment<2>(2 * i) << scale[2 * i] * 0.225,
        scale[2 * i + 1] * 0.125;
    //    leg_position.setReference(i, ref);
  }

  while (robot.isRunning())
  {
    leg_tracking = leg_position.getError();
    steering_error = leg_orientation_z.getError();
    for (int i = 0; i < 4; i++)
      log_position.segment<2>(2 * i) =
          leg_position.getReferenceWorld(i, true).head(2);

    log_pelvis = pelvis_position.getReference();
    next_position << r* std::cos(t - pi2) + initial_state[0],
        r * std::sin(t - pi2) + r + initial_state[1], // orientation;
        std::atan2(std::sin(t), std::cos(t)) + orientation;

    current_position = leg_position.getState().head(3);
    next_step = (next_position - current_position);

    next_step[2] -= 6.28318531 * std::floor((next_step[2] + 3.14159265) /
                                            6.28318531); // normalize to -pi:pi

    next_step = next_step / dt;

    leg_tracking = leg_position.getError();

    // converge
    if (path > 200 && ds >= ds_max && mode == 0){
//    if (mode == 0){
      bool done = true;

      for (int i = 0; i < 4; i++)
      {
        leg_sp = leg_position.getReference(i);
        double error = leg_sp[0] - (scale[2 * i] * r_sp* std::cos(st) + leg_init[i * 2 + 0]);
        if (std::fabs(error) > ds_step){
//          std::cout << "1: " << error << std::endl;
          done = false;
          if (error > 0)
            leg_sp[0] -= ds_step*2;
          else if (error < 0)
            leg_sp[0] += ds_step*2;
        }
        else
            leg_sp[0] = scale[2 * i] * r_sp* std::cos(st) + leg_init[i * 2 + 0];

        error = leg_sp[1] - (-scale[2 * i + 1] * r_sp * std::sin(st) + leg_init[i * 2 + 1]);

        if (std::fabs(error) > ds_step){
//          std::cout << "2: " << error << std::endl;
          done = false;
          if (error > 0)
            leg_sp[1] -= ds_step*2;
          else if (error < 0)
            leg_sp[1] += ds_step*2;
        }
        else
            leg_sp[1] = (-scale[2 * i + 1] * r_sp * std::sin(st) + leg_init[i * 2 + 1]);

        leg_position.setReference(i, leg_sp);
      }
      if (done) {mode++; path = 0;}
    }

//    // circular trajectory
    if (path > 500 && mode == 1)
//      if (mode == 1)
    {
//      std::cout << 2 << std::endl;
      if (sp > st || sp < -80 * 3.1416 / 180)
      {
        if (wait < 500)
        {
          std::cout << "wait: " << wait << std::endl;
          dsp = 0;
          wait++;
        }
        else
        {
          dsp = ((sp-st) > 0) ? -dss : dss;
          z += 0.005;
          wait = 0;
        }
      }

      for (int i = 0; i < 4; i++)
        log_position.segment<2>(2 * i) =
            leg_position.getReferenceWorld(i, true).head(2);

      log_pelvis = pelvis_position.getReference();
      // step reference
      for (int i = 0; i < 4; i++)
      {
        leg_sp << scale[2 * i] * r_sp* std::cos(sp) + leg_init[i * 2 + 0],
            -scale[2 * i + 1] * r_sp * std::sin(sp) + leg_init[i * 2 + 1];

        leg_position.setReference(i, leg_sp);
//        std::cout << i << "\t" << leg_sp << std::endl;
      }
      sp += dsp;
    }
    else if (mode == 1) {path++; std::cout << "wait for circle: " << path << std::endl;}

    events::combined2(robot, leg_position, steerings, next_step, 0.4, 0.6, dt, 0.01);

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

    if (leg_tracking.cwiseAbs().maxCoeff() < 0.05)
    {
      path++;
    }
    else if (ds < ds_max)
      path = 0;

    if (path > 500 && ds < ds_max)
    {
      ds += ds_step;
      path = 0;
      std::cout << "path\t" << ds  << "/" << ds_max << std::endl;
    }

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
    double rate, const double dt, const double& z,
    mwoibn::VectorN log_position, mwoibn::VectorN log_pelvis)
{

  mwoibn::VectorN command = hierarchical_controller.update();

  //  for (int i = 0; i < command.size(); i++)
  //    std::cout << command[i] << "\t";
  //  std::cout << std::endl;

  command = command * 1/rate +
            robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

  robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);

//  robot.controllers.send();

  trackerUpdater(tracker, pelvis_position, leg_position, z, log_position,
                 log_pelvis);

  robot.update();
}
