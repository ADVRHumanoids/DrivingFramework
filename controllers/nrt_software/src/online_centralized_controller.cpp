#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <custom_messages/CustomCmnd.h>
#include "mgnss/controllers/online_centralized_controller.h"
#include <config.h>

void switchMode(mwoibn::robot_class::Robot* robot_ptr,
                mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{

        if (*motor_side)
        {
                robot_ref_ptr->state.set(
                        robot_ptr->command.get(mwoibn::robot_class::INTERFACE::POSITION),
                        mwoibn::robot_class::INTERFACE::POSITION);
                robot_ref_ptr->update();
        }
        else
                robot_ptr->command.set(
                        robot_ref_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION),
                        mwoibn::robot_class::INTERFACE::POSITION);
}

void setReference(RigidBodyDynamics::Math::VectorNd* state_ptr,
                  mwoibn::robot_class::Robot* robot_ptr,
                  mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{
        if (*motor_side)
        {
                robot_ref_ptr->state.set(*state_ptr,
                                         mwoibn::robot_class::INTERFACE::POSITION);
                robot_ref_ptr->update();
        }
        else
                robot_ptr->command.set(*state_ptr,
                                       mwoibn::robot_class::INTERFACE::POSITION);
}

bool setMotorSideReference(std_srvs::SetBool::Request& req,
                           std_srvs::SetBool::Response& res,
                           mwoibn::robot_class::Robot* robot_ptr,
                           mwoibn::robot_class::Robot* robot_ref_ptr,
                           bool* motor_side)
{
        std::string enabled = "motor_side", disabled = "link_side";
        if (*motor_side == req.data)
        {
                res.success = true;
                res.message = "Reference has alredy been on " +
                              (*motor_side ? enabled : disabled) + "!";
                return true;
        }

        *motor_side = req.data;
        res.message =
                "Reference is now for " + (*motor_side ? enabled : disabled) + "!";
        switchMode(robot_ptr, robot_ref_ptr, motor_side);
        res.success = true;
        return true;
}

bool resetReference(std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res,
                    mwoibn::robot_class::Robot* robot_ptr,
                    mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{
        RigidBodyDynamics::Math::VectorNd state =
                robot_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION);
        setReference(&state, robot_ptr, robot_ref_ptr, motor_side);
        return true;
}

void getContacts(const custom_messages::CustomCmnd::ConstPtr& msg,
                 mwoibn::robot_class::Robot* robot_ptr,
                 mwoibn::robot_class::Robot* robot_ref_ptr)
{

        for (int i = 0; i < robot_ptr->contacts().size(); i++)
        {
                //    std::cout << "for" << std::endl;

                if (msg->onlineGain1[i])
                {
                        //      std::cout << "active" << std::endl;

                        robot_ptr->contacts().contact(i).activate();
                        robot_ref_ptr->contacts().contact(i).activate();
                }
                else
                {
                        //      std::cout << "not active" << std::endl;
                        robot_ptr->contacts().contact(i).deactivate();
                        robot_ref_ptr->contacts().contact(i).deactivate();
                }
        }

        return;
}

int main(int argc, char** argv)
{

        ros::init(argc, argv, "centralized_controller"); // initalize node

        ros::NodeHandle n;
        bool motor_side = false;
        std::string path = std::string(DRIVING_FRAMEWORK_WORKSPACE);
        // init real robot
        mwoibn::robot_class::RobotXBotNRT robot(path+"DrivingFramework/configs/mwoibn/configs/mwoibn_2_5.yaml",
                                                "default", path+"DrivingFramework/configs/mwoibn/configs/support/centralized_controller.yaml");
        mwoibn::robot_class::RobotXBotNRT robot_ref(
                path+"DrivingFramework/locomotion_framework/configs/"
                "mwoibn_v2.yaml",
                "reference", path+"DrivingFramework/controllers/nrt_software/configs/centralized_controller.yaml");

        mgnss::controllers::OnlineCentralizedController controller(robot);

        ros::Subscriber sub = n.subscribe<custom_messages::CustomCmnd>(
                "CoM_regulator/contacts", 1,
                boost::bind(&getContacts, _1, &robot, &robot_ref));
        //  // debbuging purposes, switch between motor side and link side reference
        ros::ServiceServer switch_reference =
                n.advertiseService<std_srvs::SetBool::Request,
                                   std_srvs::SetBool::Response>(
                        "centralized_controller/motor_side_reference",
                        boost::bind(&setMotorSideReference, _1, _2, &robot, &robot_ref,
                                    &motor_side));

        // debbuging purposes, set current robot position as new reference
        ros::ServiceServer reset_reference =
                n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                        "centralized_controller/reset_reference",
                        boost::bind(&resetReference, _1, _2, &robot, &robot_ref,
                                    &motor_side));

        robot.get();

        // set valid reference for current contact points
        robot.command.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                          mwoibn::robot_class::INTERFACE::POSITION);
        robot_ref.state.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                            mwoibn::robot_class::INTERFACE::POSITION);

        while (ros::ok())
        {
                if (robot_ref.get())
                        controller.fullUpdate(robot_ref.state.get(mwoibn::robot_class::INTERFACE::POSITION), robot_ref.state.get(mwoibn::robot_class::INTERFACE::VELOCITY));
        }
}
