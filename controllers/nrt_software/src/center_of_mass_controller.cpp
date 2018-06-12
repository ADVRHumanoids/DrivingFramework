#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/communication_modules/ros_basic.h>
#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/hierarchical_control/tasks/joint_positions_task.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>
#include <mwoibn/point_handling/robot_points_handler.h>
//#endif

#include <custom_services/updatePDGains.h>

// trajectory generation
#include <mwoibn/reference_generation/line.h>
#include <mwoibn/reference_generation/local_circle.h>

// temp
#include <MathGeoLib/Algorithm/Random/LCG.h>
#include <MathGeoLib/Geometry/GeometryAll.h>

bool changeGain(
        custom_services::updatePDGains::Request& req,
        custom_services::updatePDGains::Response& res,
        mwoibn::hierarchical_control::controllers::Default* controller)
{
        res.success = controller->updateGain(req.nr, req.p);
        if (res.success)
                res.message =
                        "Gain for task " + std::to_string(req.nr) + " changed succesfully";
        else
        {
                res.message = "Couldn't change gain for a task " + std::to_string(req.nr);
        }

        return true;
}

bool resetReference(
        mwoibn::robot_class::Robot* robot_ptr,
        mwoibn::hierarchical_control::tasks::CenterOfMass* com_task_ptr,
        mwoibn::visualization_tools::RvizTrackPoint* tracker_ptr)
{

        Polygon support_polygon;
        for (auto& cp : robot_ptr->contacts().getPositions())
                support_polygon.p.push_back(
                        mwoibn::reference_generation::Utils::toFloat3(cp.head(3)));

        RigidBodyDynamics::Math::Vector3d com_point =
                mwoibn::reference_generation::Utils::fromFloat3(
                        support_polygon.CenterPoint());
        com_task_ptr->setReference(com_point.head(2));
        tracker_ptr->updateMarker(0, com_point[0], com_point[1], com_point[2]);

        return true;
}

int main(int argc, char** argv)
{

        ros::init(argc, argv, "center_of_mass_controller"); // initalize node

        ros::NodeHandle n;

        // load robot configuration and set it up to the starting position
//  mwoibn::robot_class::RobotRosNRT robot("/robot_description", "/robot_semantic_description", true, true,
//                                         "joint_states", "/gazebo/link_states",
//                                         false, false);

//  robot.controllers.add(
//      std::unique_ptr<mwoibn::communication_modules::BasicModule>(
//          new mwoibn::communication_modules::RosController(
//              robot.command, robot.getDofs(), "position_controller/reference",
//              true, false, false)));

        mwoibn::robot_class::RobotRosNRT robot("/home/user/malgorzata/workspace/src/locomotion_framework/configs/ros.yaml","higher_scheme");

        RigidBodyDynamics::Math::VectorNd command =
                Eigen::VectorXd::Zero(robot.getDofs());

        for (int i = 0; i < robot.contacts().size(); i++)
        {
                robot.contacts().contact(i).activate();
        }

        ros::Rate rate(100);
        robot.update();

        // Set-up hierachical controller
        mwoibn::hierarchical_control::tasks::CenterOfMass com_task(robot);
        mwoibn::hierarchical_control::tasks::Constraints constraints_task(robot);

        Eigen::VectorXd pelvis(3);

        pelvis << 0, 0, 1;

        mwoibn::hierarchical_control::tasks::CartesianSelective pelvis_hight(
                mwoibn::point_handling::PositionsHandler("ROOT", robot, {"pelvis"}),
                pelvis);

        pelvis << 0, 0, 0.4;

        pelvis_hight.setReference(pelvis);

        // Init controller
        mwoibn::hierarchical_control::controllers::Default hierarchical_controller;

        // ros topics/service support
        ros::ServiceServer service =
                n.advertiseService<custom_services::updatePDGains::Request,
                                   custom_services::updatePDGains::Response>(
                        "change_task_gain",
                        boost::bind(&changeGain, _1, _2, &hierarchical_controller));


        // Set tasks
        hierarchical_controller.addTask(constraints_task, static_cast<unsigned int>(0), 1);
        hierarchical_controller.addTask(com_task, static_cast<unsigned int>(0), 100);
        hierarchical_controller.addTask(pelvis_hight, static_cast<unsigned int>(0), 25);

        //#ifdef VISUALIZATION_TOOLS
        // create trackers classes
        mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");
        tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE, "world",
                           0.002, 0.002,
                           0.002); // nr.0 - CoM reference trajectory
        tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - actual CoM
        //  tracker.initMarker(mwoibn::visualization_tools::Utils::TYPE::POINT,
        //  "world",
        //  0.003, 0.003,
        //                     0.003); // nr.2 - current CoM reference
        mwoibn::visualization_tools::RvizTrackPoint tracker_contacts(
                "rviz/contacs_tracker");

        tracker_contacts.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE,
                                    "world", 0.004, 0.004, 0.004);
        tracker_contacts.setColor(0, 1, 0, 0);

        //  mwoibn::point_handling::PositionsHandler com_trackers("ROOT", robot,
        //  {"pelvis", "pelvis"}, {P_0, RigidBodyDynamics::Math::Vector3d(0,0,0)});
        Eigen::Vector3d com_point;

        //#endif
        //#ifdef VISUALIZATION_TOOLS

        // create com reference
        robot.centerOfMass().update();
        com_point = robot.centerOfMass().get();
        double max_step = 0.0001;
        Eigen::VectorXd change(3);
        Eigen::VectorXd original(3);
        original << com_point.head(2), 0;
        int direction = -1;

        change << 0.15, 0.02, 0;

        Eigen::Vector3d one, two, three, test_point;
        double radious = 0.06;
        one << -radious, 0, 0;
        two << radious, 0, 0;
        three << radious, radious, 0;

//    mwoibn::reference_generation::Line ref_com(original, max_step, original +
//    change);
        mwoibn::reference_generation::Local_Circle ref_com(
                original + one, original + two, original + three, max_step);

        while (ros::ok())
        {
//        resetReference(&robot, &com_task, &tracker);

                command =
                        hierarchical_controller.update() * rate.expectedCycleTime().toSec() +
                        robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

                robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
                robot.controllers.send();
                com_task.setReference(ref_com.nextStep().head(2));


                if (ref_com.isDone())
                {
//              ref_com.setFinalState(original + direction * change);
                        ref_com.setFinalState(ref_com.getFinalState() + 3.14);

//      direction = -direction;
                }

                //  #ifdef VISUALIZATION_TOOLS

                com_point = ref_com.getCurrentPoint();
                tracker.updateMarker(0, com_point[0], com_point[1], com_point[2]);

                com_point.head(2) = robot.centerOfMass().get().head(2);
                com_point[2] = 0;
                tracker.updateMarker(1, com_point[0], com_point[1], com_point[2]);

                tracker.publish();
//    if (robot.contacts().sizeActive())
//    {
//      for (auto& cp : robot.contacts().getPositions())
//        tracker_contacts.updateMarker(0, cp[0], cp[1], cp[2]);
//      tracker_contacts.updateMarker(0, robot.contacts().getPositions().at(0)[0],
//                                    robot.contacts().getPositions().at(0)[1],
//                                    robot.contacts().getPositions().at(0)[2]);
//    }
//    tracker_contacts.publish();
//    tracker_contacts.reset(0);

                //  #endif

                robot.update();
        }
}
