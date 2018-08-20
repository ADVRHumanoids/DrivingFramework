#include <config.h>
#include <mwoibn/loaders/robot.h>
#include "mgnss/controllers/wheeled_motion_event.h"

#include "mgnss/controllers/wheeled_references_v3.h"
#include <custom_services/updatePDGains.h>
#include <custom_messages/CustomCmnd.h>


bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support);



int main(int argc, char** argv)
{
        ros::init(argc, argv, "wheels_reference"); // initalize node

        ros::NodeHandle n;

// init wheels_controller

        mwoibn::loaders::Robot loader;
        std::string config_file = std::string(DRIVING_FRAMEWORK_WORKSPACE) +
                                  "DrivingFramework/"
                                  "locomotion_framework/configs/mwoibn_v2.yaml";
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["wheels_support"];

        std::string secondary_file = "";
        if (config["secondary_file"])
                secondary_file = config["secondary_file"].as<std::string>();

//  mwoibn::robot_class::Robot& robot = loader.init(
//      std::string(DRIVING_FRAMEWORK_WORKSPACE) +
//          "DrivingFramework/"
//          "locomotion_framework/configs/mwoibn_v2.yaml",
//      "fake", std::string(DRIVING_FRAMEWORK_WORKSPACE) +
//                     "DrivingFramework/"
//                     "locomotion_framework/configs/lower_body.yaml");

        mwoibn::robot_class::Robot& robot = loader.init(config_file, config["robot"].as<std::string>(), secondary_file);



        mwoibn::SupportPolygon3 support(0.45, 0.22, 0.078, robot.rate());

//  mgnss::controllers::WheeledMotionEvent wheeld_controller(robot);
        mgnss::controllers::WheeledMotionEvent wheeld_controller(robot, config_file);

        wheeld_controller.init();
        // starting

        mwoibn::VectorN base(12);

        base << 0.15, 0.125, 0.0, 0.15, -0.125, 0.0, -0.25, 0.125, 0.0, -0.25, -0.125,
        0.0;
        support.setBase(base);
        support.setUpperLimit(-15 * 3.1416 / 180);

        support.setLowerLimit(-65 * 3.1416 / 180);
        support.setRadious(0.38);
        support.setStep(0.002);

        support.setCurrent(wheeld_controller.getSupportReference());
        support.setDesired(wheeld_controller.getSupportReference());

        ros::ServiceServer service =
                n.advertiseService<custom_services::updatePDGains::Request,
                                   custom_services::updatePDGains::Response>(
                        "wheels_support_cmnd",
                        boost::bind(&evenstHandler, _1, _2, &support));

        ros::Publisher publish = n.advertise<custom_messages::CustomCmnd>("wheels_support",1);

        custom_messages::CustomCmnd msg;
        msg.position.resize(13);
        msg.position[12] = mwoibn::IS_VALID;

        while (ros::ok())
        {
                support.update();
                for(int i = 0; i < 12; i++)
                        msg.position[i] = support.get()[i];
                publish.publish(msg);

                ros::spinOnce();
                robot.wait();
//	std::cout << "support\t" << support.get().transpose() << std::endl;
        }
        msg.position[12] = mwoibn::INVALID;
        publish.publish(msg); //???

}

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support)
{


        if (req.p == 1)
        { // support
                mwoibn::SUPPORT_MOTION motion;
                mwoibn::SUPPORT_STATE state;

                if (req.d >= 0 && req.d < 3)
                {
                        motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

                        if (req.nr > 0 && req.nr < 4)
                        {
                                state = static_cast<mwoibn::SUPPORT_STATE>(req.nr);
                                support->initMotion(motion, state);
                        }
                        else{
                                support->changeMotion(motion);
                        }

                        support->setModeX(0, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeX(1, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeX(2, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeX(3, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeY(0, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeY(1, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeY(2, mwoibn::SUPPORT_INTERFACE::POSITION);
                        support->setModeY(3, mwoibn::SUPPORT_INTERFACE::POSITION);
                        res.success = true;
                        return true;
                }
                else if (req.d == 3)
                {
                        support->setStep(req.nr / 10000.0);
                }
                else if (req.d == 4)
                {
                        support->setUpperLimit(req.nr / 100.0);
                }
                else if (req.d == 5)
                {
                        support->setLowerLimit(req.nr / 100.0);
                }
                else if (req.d == 6)
                {
                        support->setRadious(req.nr / 100.0);
                }
                else
                {
                        res.success = false;
                        return false;
                }
                res.success = true;
                return true;
        }
        else if (req.p == 2)
        {
//	std::cout << req.p << "\t" << req.d << std::endl;
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
                support->setVelX(req.d, req.nr / 1000.0);
        }
        else if (req.p == 3)
        {
//	std::cout << req.p << "\t" << req.d << std::endl;

                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
                support->setVelY(req.d, req.nr / 1000.0);
        }
        else if (req.p == 4)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
//    controller->claim(req.d);
                support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
                support->setVelZ(req.d, req.nr / 1000.0);
        }
        else if (req.p == 5)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setVelX(req.d, 0.0);
                support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else if (req.p == 6)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setVelY(req.d, 0.0);
                support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else if (req.p == 7)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
//    controller->release(req.d);
                support->setVelZ(req.d, 0.0);
                support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else if (req.p == 8)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setDesX(req.d, req.nr / 100.0);
                support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else if (req.p == 9)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setDesY(req.d, req.nr / 100.0);
                support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else if (req.p == 10)
        {
                if (req.d > 3 || req.d < 0)
                {
                        res.success = false;
                        return false;
                }
                support->setDesZ(req.d, req.nr / 100.0);
                support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
        }
        else
        {
                res.success = false;
                return false;
        }

        res.success = true;
        return true;
}
