#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion_full.h>
#include <mgnss/controllers/wheeled_references.h>
#include <custom_services/updatePDGains.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::hierarchical_control::tasks::CartesianSelective* position,
                   mwoibn::hierarchical_control::tasks::CastorAngleTask* castor, mwoibn::hierarchical_control::tasks::CamberAngleTask *camber, mwoibn::hierarchical_control::tasks::SteeringAngleTask* steer);

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

        mwoibn::hierarchical_control::tasks::CartesianSelective leg_position(
                mwoibn::point_handling::PositionsHandler("ROOT", robot,
                                                         robot.getLinks("wheels")),
                selection);

        mwoibn::Axis x, y, z, ax;
        z << 0, 0, -1;
        y << 0, 1, 0;
        x << -1, 0, 0;
        ax << 0, 0, 1;

        mwoibn::hierarchical_control::tasks::CastorAngle castor1(
                robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::CamberAngle camber1(
                robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::SteeringAngle steer1(
                robot, mwoibn::point_handling::Point("ankle2_1", robot.getModel()), x, y,
                z, ax);
        z << 0,  0, -1;
        y << 0, -1,  0;
        x << 1,  0,  0;
        mwoibn::hierarchical_control::tasks::CastorAngle castor2(
                robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::CamberAngle camber2(
                robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::SteeringAngle steer2(
                robot, mwoibn::point_handling::Point("ankle2_2", robot.getModel()), x, y,
                z, ax);
        z << 0,  0, -1;
        y << 0, -1,  0;
        x << 1,  0,  0;
        mwoibn::hierarchical_control::tasks::CastorAngle castor3(
                robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::CamberAngle camber3(
                robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
                z, ax);
        robot, mwoibn::point_handling::Point("ankle2_3", robot.getModel()), x, y,
        mwoibn::hierarchical_control::tasks::SteeringAngle steer3(
                z, ax);
        z <<  0,  0, -1;
        y <<  0,  1,  0;
        x << -1,  0,  0;
        robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
        mwoibn::hierarchical_control::tasks::CastorAngle castor4(
                z, ax);
        mwoibn::hierarchical_control::tasks::CamberAngle camber4(
                robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
                z, ax);
        mwoibn::hierarchical_control::tasks::SteeringAngle steer4(
                robot, mwoibn::point_handling::Point("ankle2_4", robot.getModel()), x, y,
                z, ax);

        mwoibn::hierarchical_control::tasks::CastorAngleTask castor({castor1, castor2, castor3, castor4}, robot);
        mwoibn::hierarchical_control::tasks::CamberAngleTask camber({camber1, camber2, camber3, camber4}, robot);
        mwoibn::hierarchical_control::tasks::SteeringAngleTask steer({steer1, steer2, steer3, steer4}, robot);
        mwoibn::hierarchical_control::tasks::Constraints constraints(robot);

        mwoibn::VectorN command;

        int ratio = 1;
        double damp = 1e-4;
        // Set initaial HC tasks
        mwoibn::hierarchical_control::controllers::Default hierarchical_controller;

//  hierarchical_controller.addTask(constraints, gain, task, damp);
//  task++;
//  gain << 50 * ratio;
//  hierarchical_controller.addTask(pelvis, gain, task, 1e-3);
//  task++;
//  gain << 50 * ratio;
//  hierarchical_controller.addTask(pelvis_orn, gain, task, 1e-3);
//  task++;
        gain << 50 * ratio;
        hierarchical_controller.addTask(steer, 50*ratio, 1e-3);
        task++;
//  gain << 25 * ratio;
//  hierarchical_controller.addTask(castor, gain, task, 1e-3);
//  task++;
//  gain << 25 * ratio;
//  hierarchical_controller.addTask(camber, gain, task, 0.04);
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
        mwoibn::hierarchical_control::tasks::CartesianSelective* position,
        mwoibn::hierarchical_control::tasks::CastorAngleTask* castor,
        mwoibn::hierarchical_control::tasks::SteeringAngleTask* steer)
mwoibn::hierarchical_control::tasks::CamberAngleTask* camber,
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
