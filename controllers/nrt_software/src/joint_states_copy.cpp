#include <config.h>

#include <mwoibn/loaders/robot.h>

#include "mgnss/controllers/joint_states.h"

#include <custom_services/jointStateCmnd.h>


bool referenceHandler(custom_services::jointStateCmnd::Request& req,
                      custom_services::jointStateCmnd::Response& res,
                      mgnss::controllers::JointStates *controller);

int main(int argc, char** argv)
{

        ros::init(argc, argv,
                  "joint_state"); // initalize node needed for the service

        ros::NodeHandle n;

        std::string path = std::string(DRIVING_FRAMEWORK_WORKSPACE);

        mwoibn::loaders::Robot loader;

        mwoibn::robot_class::Robot& robot = loader.init(path+"DrivingFramework/locomotion_framework/configs/mwoibn_v2_5.yaml", "joint_space");

        mgnss::controllers::JointStates controller(robot);

        controller.init();
        ros::ServiceServer trajectory_service =
                n.advertiseService<custom_services::jointStateCmnd::Request,
                                   custom_services::jointStateCmnd::Response>(
                        "trajectory", boost::bind(&referenceHandler, _1, _2, &controller));


        while(ros::ok()) {
                controller.update();
                controller.send();
        }

}

bool referenceHandler(custom_services::jointStateCmnd::Request& req,
                      custom_services::jointStateCmnd::Response& res,
                      mgnss::controllers::JointStates* controller)
{

        if(!controller->setFullPosition(req.position)) {
                res.message = "Position " + req.position + " has not been defined in the robot";
        }
        //controller->setVelocity(req.velocity);
        if(req.pos_step)
                controller->step(req.pos_step);
        res.success = true;
        return true;
}
