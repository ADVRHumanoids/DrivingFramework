#include "mgnss/controllers/wheeled_references.h"
#include <custom_services/updatePDGains.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base);

int main(int argc, char** argv)
{
        ros::init(argc, argv, "wheels_reference"); // initalize node

        ros::NodeHandle n;

        mwoibn::robot_class::RobotXBotNRT robot(
                "/home/user/malgorzata/workspace/src/locomotion_framework/configs/"
                "mwoibn_v2.yaml",
                "default", "/home/user/malgorzata/workspace/src/controllers/nrt_software/configs/"
                "wheels.yaml");
        mwoibn::SupportPolygon support(0.4, 0.22);
        support.setBase(0.225, 0.125);       // HARDCODED
        support.setUpperLimit(-1 * 3.1416 / 180); // HARDCODED
        support.setLowerLimit(-80 * 3.1416 / 180); // HARDCODED
        support.setRadious(0.40);            // HARDCODED

        mwoibn::Base base;

        base.heading.setUpperLimit(2 * 3.1416 / 180); // HARDCODED
        base.heading.setLowerLimit(-2 * 3.1416 / 180); // HARDCODED

        // ros topics/service support
        ros::ServiceServer service =
                n.advertiseService<custom_services::updatePDGains::Request,
                                   custom_services::updatePDGains::Response>(
                        "wheels_command",
                        boost::bind(&evenstHandler, _1, _2, &support, &base));

        Eigen::Matrix<double, 13, 1> references;
        XBot::PublisherNRT<Eigen::Matrix<double, 13, 1> > pub("wheels_reference");

        std::cout << "Initialized NRT subscriber" << std::endl;

        bool valid = false;
        while (robot.isRunning() && !valid)
        {
                std::cout << "Waiting for state feedback" << std::endl;
                valid = robot.get();
        }

        // starting
        base.setBasePosition(
                robot.state.position.get().head(3));
        base.pose.setCurrent(robot.state.position.get().head(2)); // change it to the data from robot

        base.height.setCurrent(0.42);
        base.heading.setCurrent(0);

        support.setCurrent(0.50, 0.22);

        support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT,
                           mwoibn::SUPPORT_STATE::DEFAULT);

        base.initMotion(mwoibn::BASE_MOTION::STOP, mwoibn::BASE_DIRECTION::POSITIVE);

        references[12] = mwoibn::IS_VALID;

        while (robot.isRunning())
        {
                support.update();
                base.update();

                references.head(8) = support.get();
                references.segment<3>(8) = base.getPosition();
                references[11] = base.heading.get()[0];

                pub.write(references);

                std::cout << "references" << std::endl;
                std::cout << references << std::endl;

                robot.update();
        }
}

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base)
{
        bool correct = true;
        if (req.p == 1) // base
        {
                mwoibn::BASE_MOTION motion;
                mwoibn::BASE_DIRECTION direction;
                if (req.d > 0 && req.d < 3)
                {
                        motion = static_cast<mwoibn::BASE_MOTION>(req.d);

                        if (req.nr > 1 || req.nr < 0)
                                correct = false;
                        else
                                direction = static_cast<mwoibn::BASE_DIRECTION>(req.nr);
                        if (correct)
                                base->initMotion(motion, direction);
                }
                else if (req.d == 3)
                {
                        base->heading.setStep(req.nr / 10000.0);
                        base->pose.setStep(req.nr / 10000.0);
                }
                else if (req.d == 4)
                {
                        base->heading.setUpperLimit(req.nr / 100.0);
                }
                else if (req.d == 5)
                {
                        base->heading.setLowerLimit(req.nr / 100.0);
                }
                else if (req.d == 6)
                {
                        base->pose.setRadious(req.nr / 100.0);
                }
                else if (req.d == 7)
                {
                        base->height.setCurrent(req.nr / 100.0);
                }
                else
                        correct = false;
        }
        else if (req.p == 2) // support
        {
                mwoibn::SUPPORT_MOTION motion;
                mwoibn::SUPPORT_STATE state;
                if (req.d > 0 && req.d < 3)
                {
                        motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

                        if (req.nr > 2 || req.nr < 0)
                                correct = false;
                        else
                                state = static_cast<mwoibn::SUPPORT_STATE>(req.nr);
                        if (correct)
                                support->initMotion(motion, state);
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
                        correct = false;
        }
        else
                correct = false;

        res.success = correct;
        return correct;
}
