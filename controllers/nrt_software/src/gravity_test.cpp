#include <config.h>

#include <mwoibn/loaders/robot.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/hierarchical_control/tasks/joint_positions_task.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/controller_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_world_task.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>

#include <mwoibn/point_handling/robot_points_handler.h>
//#endif
#include <custom_services/updateGains.h>
#include <custom_services/updatePrint.h>
#include <custom_messages/CustomCmnd.h>

// trajectory generation
//#include <mwoibn/reference_generation/line.h>
//#include <mwoibn/reference_generation/local_circle.h>
//#include "mgnss/communication/basic_handler.h"

// temp
//#include <MathGeoLib/Algorithm/Random/LCG.h>
//#include <MathGeoLib/Geometry/GeometryAll.h>

#include <mwoibn/eigen_utils/eigen_utils.h>

bool wheelHandler(custom_services::updateGains::Request& req,
                  custom_services::updateGains::Response& res, double* x,
                  double* y, double* z, double* th, bool* reset);

int main(int argc, char** argv)
{

        ros::init(argc, argv,
                  "gravity_test"); // initalize node needed for the service

        ros::NodeHandle n;

        mwoibn::loaders::Robot loader;
        mwoibn::robot_class::Robot& robot = loader.init(
                std::string(DRIVING_FRAMEWORK_WORKSPACE) +
                "DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml",
                "default");

        robot.wait();
        robot.get();
        robot.updateKinematics();

        RigidBodyDynamics::Math::VectorNd command =
                Eigen::VectorXd::Zero(robot.getDofs());

        mwoibn::hierarchical_control::tasks::Constraints constraints_task(robot);

        Eigen::Vector3d pelvis;
        pelvis << 1, 1, 1;

        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", robot,
                                                           robot.getLinks("base"));
        mwoibn::hierarchical_control::tasks::CartesianSelective pelvis_hight(pelvis_ph,
                                                                             pelvis);

        RigidBodyDynamics::Math::Vector3d axis;
        axis << 1, 1, 1;
        mwoibn::point_handling::OrientationsHandler pelvis_orn(
                "ROOT", robot, robot.getLinks("base"));
        mwoibn::hierarchical_control::tasks::OrientationSelective pelvis_orientation(
                pelvis_orn, axis, robot);

        axis << 0, 0, 1;
        pelvis_orientation.setReference(
                0, pelvis_orientation.getOffset(0) *
                pelvis_orientation.points().getPointStateWorld(0));
        //  pelvis_orientation.setReference(0,
        //  RigidBodyDynamics::Math::Quaternion(0,0,0,1));
        mwoibn::hierarchical_control::controllers::Default hierarchical_controller;

        Eigen::Vector3d com_point = pelvis_ph.getPointStateWorld(0);
        double z = 0;
        double y = 0;
        double x = 0;
        double th = 0;
        bool reset = false;
        std::cout << "init" << com_point << std::endl;

        ros::ServiceServer trajectory_service =
                n.advertiseService<custom_services::updateGains::Request,
                                   custom_services::updateGains::Response>(
                        "trajectory",
                        boost::bind(&wheelHandler, _1, _2, &x, &y, &z, &th, &reset));

        // Set initaial HC tasks
        hierarchical_controller.addTask(constraints_task, 0, 1, 1e-6);
        hierarchical_controller.addTask(pelvis_hight, 1, 50, 1e-6);
        hierarchical_controller.addTask(pelvis_orientation, 2, 50, 1e-6);

        //#ifdef VISUALIZATION_TOOLS
        // create trackers
        //  mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");
        //  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT,
        //  "world",
        //                     0.002, 0.002,
        //                     0.002); // nr.0 - Pelvis reference trajectory
        //  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT,
        //  "world",
        //                     0.004, 0.004,
        //                     0.004); // nr.1 - actual pelvis position

        //#endif
        //#ifdef VISUALIZATION_TOOLS
        //  robot.command.position.set(robot.state.position.get());
        //    std::cout << "com_point\n" << com_point << std::endl;

//   double eps = 0.005;
        std::cout << "orientation reference\n " << pelvis_orientation.getReference(0)
                  << std::endl;
        std::cout << "orientation offset\n " << pelvis_orientation.getOffset(0)
                  << std::endl;

        while (ros::ok())
        {

                //    std::cout << "com_point" << std::endl;
                //    std::cout << com_point << std::endl;

                com_point[0] += x * robot.rate();
                com_point[1] += y * robot.rate();
                com_point[2] += z * robot.rate();

//    if (reset)
//    {
//      com_point = pelvis_ph.getPointStateWorld(0);
//      reset = false;
//    }
                pelvis_hight.setReference(com_point);

                pelvis_orientation.setReference(
                        0, pelvis_orientation.getOffset(0) *
                        mwoibn::Quaternion::fromAxisAngle(axis, th));

                command = hierarchical_controller.update();

                robot.command.velocity.set(command);

                command = command * robot.rate() +
                          robot.state.position.get();

                //    std::cout << "base" << std::endl;
                //    std::cout <<
                //    robot.state.position.get().head(6) <<
                //    std::endl;

                robot.command.position.set(command);

                robot.update();
        }
}

bool wheelHandler(custom_services::updateGains::Request& req,
                  custom_services::updateGains::Response& res, double* x,
                  double* y, double* z, double* th, bool* reset)
{

        *x = req.a_p / 1000.0;
        *y = req.a_d / 1000.0;
        *z = req.j_p / 1000.0;
        *th = req.j_d / 1000.0;
        *reset = true;
        res.success = true;
        return true;
}
