#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/hierarchical_control/tasks/joint_positions_task.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_world_task.h>
#include <mwoibn/hierarchical_control/tasks/controller_task.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/tasks/orientation_selective_task.h>

//#ifdef VISUALIZATION_TOOLS
#include <mwoibn/visualization_tools/rviz_track_point.h>
#include <mwoibn/visualization_tools/rviz_joint_limits.h>

#include <mwoibn/point_handling/robot_points_handler.h>
//#endif
#include <custom_services/updatePDGains.h>
#include <custom_services/updatePrint.h>
#include <custom_messages/CustomCmnd.h>

// trajectory generation
#include <mwoibn/reference_generation/line.h>
#include <mwoibn/reference_generation/local_circle.h>

// temp
#include <MathGeoLib/Algorithm/Random/LCG.h>
#include <MathGeoLib/Geometry/GeometryAll.h>

#include "mgnss/communication/events_handler.h"

bool changeGain(
        custom_services::updatePDGains::Request& req,
        custom_services::updatePDGains::Response& res,
        mwoibn::hierarchical_control::controllers::Default* controller);

bool eventHandler(custom_services::updatePrint::Request& req,
                  custom_services::updatePrint::Response& res,
                  events::EventHandler* events);

//void quternionHemisphere(RigidBodyDynamics::Math::Quaternion original,
//                         RigidBodyDynamics::Math::Quaternion& desired);

// bool quternionHemisphereOne(int i, RigidBodyDynamics::Math::Quaternion
// original,
//                            RigidBodyDynamics::Math::Quaternion& desired);

int main(int argc, char** argv)
{
        static const mwoibn::visualization_tools::Utils::TYPE tracker_type =
                mwoibn::visualization_tools::Utils::TYPE::LINE;
        ros::init(argc, argv, "center_of_mass_controller"); // initalize node

        ros::NodeHandle n;

        ros::Publisher pub =
                n.advertise<custom_messages::CustomCmnd>("CoM_regulator/contacts", 1);
        custom_messages::CustomCmnd des_q;
        // load robot configuration and set it up to the starting position

        mwoibn::robot_class::RobotRosNRT robot("/home/user/malgorzata/workspace/src/locomotion_framework/configs/ros.yaml","higher_scheme");

        // prepere vector where command is stored
        RigidBodyDynamics::Math::VectorNd command =
                Eigen::VectorXd::Zero(robot.getDofs());

        mwoibn::VectorBool active_contacts;
        active_contacts.setConstant(4, true);

        des_q.onlineGain1.resize(robot.contacts().size(), true);

        // init current set-up
        ros::Rate rate(100);
        ros::spinOnce();
        rate.sleep();

        // Set-up hierachical controller
        mwoibn::hierarchical_control::tasks::CenterOfMass com_task(robot);
        mwoibn::hierarchical_control::tasks::Constraints constraints_task(robot);

        // Initialize the legs tracking tasks
        Eigen::Vector3d P_0;
        P_0 << 0.0, 0.0, 0.0; // this could be taken from contact definition
        // // this point also should be automatic

        mwoibn::point_handling::PositionsHandler ik_legs(
                robot.getModel().GetBodyId("ROOT"), robot,
                {robot.getModel().GetBodyId("wheel_1"),
                 robot.getModel().GetBodyId("wheel_2"),
                 robot.getModel().GetBodyId("wheel_4"),
                 robot.getModel().GetBodyId("wheel_3")},
                {P_0, P_0, P_0,
                 P_0}); // this is a PH for the end-effector, shouldn't be changed

        //  mwoibn::point_handling::PositionsHandler ik_legs(
        //      robot.getModel().GetBodyId("ROOT"), robot,
        //      {robot.contacts().contact(0).point(),
        //       robot.contacts().contact(1).point(),
        //       robot.contacts().contact(2).point(),
        //       robot.contacts().contact(3).point()}); // this is a PH for the
        //       end-effector, shouldn't be changed

        mwoibn::point_handling::PositionsHandler ref_legs(
                "ROOT", robot, {"hip1_1", "hip1_2", "hip1_4", "hip1_3"});

        mwoibn::hierarchical_control::tasks::CartesianSelective legs_task(
                ik_legs, Eigen::VectorXd::Ones(12));

        Eigen::VectorXd pelvis(3);

        pelvis << 0, 0, 1;

        mwoibn::hierarchical_control::tasks::CartesianSelective pelvis_hight(
                mwoibn::point_handling::PositionsHandler("ROOT", robot, {"pelvis"}),
                pelvis);

        pelvis << 0, 0, 0.50;

        pelvis_hight.setReference(pelvis);
        //  legs_task.updateSelection(5, 1);

        //  RigidBodyDynamics::Math::VectorNd reference = legs_task.getReference();
        //  reference[2] = -0.0001;
        //  reference[5] = -0.0001;
        //  reference[8] = -0.0001;
        //  reference[11] = -0.0001;
        //  legs_task.setReference(reference);

        Eigen::Vector3d axis;
        Eigen::VectorXd selector(12);

        selector << 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0;

        mwoibn::hierarchical_control::tasks::OrientationSelective leg_orientation(
                mwoibn::point_handling::OrientationsHandler(
                        "ROOT", robot, std::vector<std::string>{"ankle2_1", "ankle2_2", "ankle2_4", "ankle2_3"}),
                selector, robot);

//  axis << 0, 1, 0;

//  RigidBodyDynamics::Math::Quaternion desired_state =
//      ;

//  quternionHemisphere(leg_orientation.getReference(0), desired_state);
//  leg_orientation.setReference(0, mwoibn::Quaternion::fromAxisAngle(axis, 3.14));
//  leg_orientation.setReference(2, mwoibn::Quaternion::fromAxisAngle(axis, 3.14));
//  axis << 1, 0, 0;
//  leg_orientation.setReference(1, mwoibn::Quaternion::fromAxisAngle(axis, 3.14));
//  leg_orientation.setReference(3, mwoibn::Quaternion::fromAxisAngle(axis, 3.14));

        mwoibn::hierarchical_control::controllers::Default hierarchical_controller;
        robot.centerOfMass().update();
        //  double max_step = 0.0005;

        //  std::vector<int> claim = {};

        events::EventHandler events(legs_task, leg_orientation, constraints_task,
                                    com_task, ref_legs, hierarchical_controller,
                                    robot);

        // ros topics/service support
        ros::ServiceServer service =
                n.advertiseService<custom_services::updatePDGains::Request,
                                   custom_services::updatePDGains::Response>(
                        "change_task_gain",
                        boost::bind(&changeGain, _1, _2, &hierarchical_controller));

        ros::ServiceServer events_service =
                n.advertiseService<custom_services::updatePrint::Request,
                                   custom_services::updatePrint::Response>(
                        "hc_event", boost::bind(&eventHandler, _1, _2, &events));

        // Set initaial HC tasks
        hierarchical_controller.addTask(constraints_task, 0, 1e-8);
        hierarchical_controller.addTask(com_task, 90, 1e-8);
//  gain << 20;
//  hierarchical_controller.addTask(leg_orientation, gain, 2,  1e-8);
        hierarchical_controller.addTask(pelvis_hight, 50, 1e-8);
        hierarchical_controller.addTask(legs_task, 10, 1e-8);




        //  gain << 5;
        //  hierarchical_controller.addTask(joint, gain, 3);

        //#ifdef VISUALIZATION_TOOLS
        // create trackers classes
        mwoibn::visualization_tools::RvizTrackPoint tracker("rviz/com_tracker");
        tracker.initMarker(tracker_type, "world",
                           0.002, 0.002,
                           0.002); // nr.0 - CoM reference trajectory
        tracker.initMarker(tracker_type, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - actual CoM
        tracker.initMarker(tracker_type, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - cartesian space point 1
        tracker.initMarker(tracker_type, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - cartesian space point 2
        tracker.initMarker(tracker_type, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - cartesian space point 3
        tracker.initMarker(tracker_type, "world",
                           0.004, 0.004,
                           0.004); // nr.1 - cartesian space point 4
        mwoibn::visualization_tools::RvizTrackPoint tracker_contacts(
                "rviz/contacs_tracker");

        tracker_contacts.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE,
                                    "world", 0.004, 0.004, 0.004); // support polygon
        tracker_contacts.initMarker(mwoibn::visualization_tools::Utils::TYPE::LINE,
                                    "world", 0.004, 0.004, 0.004); // safety_boundary
        tracker_contacts.setColor(0, 1, 0, 0);

        Eigen::Vector3d com_point;

        mwoibn::visualization_tools::RvizJointLimits joint_limits(
                "rviz/model", robot, "/robot_description");
        //#endif
        //#ifdef VISUALIZATION_TOOLS


        while (ros::ok())
        {
                command =
                        hierarchical_controller.update() * 1/100 +
                        robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

                robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
                robot.controllers.send();

                events.update();

                com_point.head(2) = com_task.getReference();
                com_point[2] = 0;

                //    //#ifdef VISUALIZATION_TOOLS
                tracker.updateMarker(0, com_point[0], com_point[1], 0);

                // trackers update
                com_point.head(2) = robot.centerOfMass().get().head(2);
                tracker.updateMarker(1, com_point[0], com_point[1], com_point[2]);

                RigidBodyDynamics::Math::VectorNd ref = legs_task.getReference();
                tracker.updateMarker(2, robot.contacts().contact(0).getPosition().head(3));
                tracker.updateMarker(3, robot.contacts().contact(1).getPosition().head(3));
                tracker.updateMarker(4, robot.contacts().contact(2).getPosition().head(3));
                tracker.updateMarker(5, robot.contacts().contact(3).getPosition().head(3));
                tracker.publish();

                RigidBodyDynamics::Math::Vector3d point;

                for (auto& contact : events.getReferenceContacts())
                {
                        point << robot.contacts().contact(contact).getPosition().head(3);
                        tracker_contacts.updateMarker(0, point[0], point[1], point[2]);
                }

                active_contacts = events.getActiveContacts();
                for (int i = 0; i < active_contacts.size(); i++) {
//      std::cout << "events:\t" << i << active_contacts[i] << std::endl;
                        des_q.onlineGain1[i] = (active_contacts[i]) ? 1 : 0;
                }

                double boundary = 0.05;
                tracker_contacts.updateMarker(1, com_point[0] + boundary,
                                              com_point[1] + boundary, com_point[2]);
                tracker_contacts.updateMarker(1, com_point[0] + boundary,
                                              com_point[1] - boundary, com_point[2]);
                tracker_contacts.updateMarker(1, com_point[0] - boundary,
                                              com_point[1] - boundary, com_point[2]);
                tracker_contacts.updateMarker(1, com_point[0] - boundary,
                                              com_point[1] + boundary, com_point[2]);

                tracker_contacts.closeLine(0);
                tracker_contacts.closeLine(1);
                tracker_contacts.publish();
                tracker_contacts.reset(0);
                tracker_contacts.reset(1);
                //    joint_limits.updateModel();
                //    joint_limits.publish();
                //    //  #endif

                pub.publish(des_q);

                robot.update();
        }
}

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

bool eventHandler(custom_services::updatePrint::Request& req,
                  custom_services::updatePrint::Response& res,
                  events::EventHandler* events)
{

        if (req.command == "raise")
        {
                res.success = events->raise(req.nr);
        }
        else if (req.command == "put")
        {
                res.success = events->put(req.nr);
        }
        else if (req.command == "claim")
        {
                res.success = events->claim(req.nr);
        }
        else if (req.command == "release")
        {
                res.success = events->release(req.nr);
        }
        else if (req.command == "reset")
        {
                res.success = events->reset();
        }
        else if (req.command == "stop")
        {
                res.success = events->stop();
        }
        else if (req.command == "circle")
        {
                res.success = events->circle(req.nr);
        }
        else if (req.command == "regain")
        {
                res.success = events->regainContact(req.nr);
        }
        else if (req.command == "break")
        {
                res.success = events->breakContact(req.nr);
        }
        else
        {
                res.message = "Unknown command";
                res.success = false;
        }
        return true;
}
