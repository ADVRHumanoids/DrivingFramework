#ifndef LOADERS_ROBOT_H
#define LOADERS_ROBOT_H

#include "mwoibn/loaders/config.h"

#include "mwoibn/robot_class/robot.h"

#if ROS
  #include "mwoibn/robot_class/robot_ros_nrt.h"
  #if XBOT
    #include "mwoibn/robot_class/robot_xbot_nrt.h"
  #endif
#endif
#if XBOT
  #include "mwoibn/robot_class/robot_xbot_rt.h"
  #include "mwoibn/robot_class/robot_xbot_feedback.h"
#endif


namespace mwoibn
{
namespace loaders
{

class Robot {
public:
Robot(){
}

Robot(std::string config_file, std::string config_name,
      std::string secondary_file = ""){
        _robot.reset(create(config_file, config_name, secondary_file).release());
}

~Robot(){
}

mwoibn::robot_class::Robot& get(){

        return *_robot;
}

mwoibn::robot_class::Robot& init(std::string config_file, std::string config_name,
                                 std::string secondary_file = ""){
        //load config
        _robot.reset(create(config_file, config_name, secondary_file).release());

        return get();
}

mwoibn::robot_class::Robot& init(YAML::Node config, std::string config_name){
        //load config
        _robot.reset(create(config, config_name).release());

        return get();
}

static YAML::Node readConfig(std::string config_file, std::string config_name,
                             std::string secondary_file = ""){

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);


        std::string error = "config file:\t" + config_file +
                            std::string("\n robot configuration: \t ") +
                            config_name + std::string("\nerror:\t");
        // check common elements
        checkConfig(config, config_name, error);

        return config;
}

static void checkConfig(YAML::Node config, std::string config_name, std::string error){

        if (!config["middleware"])
                throw std::invalid_argument(error +
                                            std::string("Please specify a middleware to be loaded."));
        if (!config["system"])
                throw std::invalid_argument(error +
                                            std::string("Please specify a system the robot is running at: hardware: HW, simulation: SM."));
        if(!config["robot"])
                throw std::invalid_argument(error +
                                            std::string("Couldn't find a robot config."));
        if(!config["robot"][config_name])
                throw std::invalid_argument(error +
                                            std::string("Unknown robot configuration: ") + config_name + std::string("."));
        if(!config["robot"]["layer"])
                throw std::invalid_argument(error +
                                            std::string("Please specify a robot layer."));
}

static std::unique_ptr<mwoibn::robot_class::Robot> create(std::string config_file, std::string config_name,
                                                          std::string secondary_file = ""){

        std::string error = "config file:\t" + config_file +
                            std::string("\n robot configuration: \t ") +
                            config_name + std::string("\nerror:\t");

        YAML::Node config = readConfig(config_file, config_name, secondary_file);

        std::string type = config["robot"]["layer"].as<std::string>();

 #if ROS
        if (config["middleware"].as<std::string>() == "ROS") {
                if (type == "NRT" || type == "ROS_NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRosNRT(config_file, config_name, secondary_file));
                else if (type == "EMPTY")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRos(config_file, config_name, secondary_file));
                else if (type == "RT")
                        throw std::invalid_argument(error +
                                                    std::string("Real time is not supported for ROS"));
                else
                        throw std::invalid_argument(error +
                                                    std::string("Unknow robot type"));
                //return *_robot;
        }
 #endif
 #if XBOT
        if (config["middleware"].as<std::string>() == "XBOT") {
                if (type == "EMPTY")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotXBotFeedback(config_file, config_name, secondary_file));
                else if (type == "RT")
                        throw std::invalid_argument(error +
                                                    std::string("Real time robot requires access to the shared memory. Please use a RT loader method "));
                //        _robot.reset(new mwoibn::robot_class::RobotXbotRT(config_file, config_name, secondary_file));
       #if ROS
                else if (type == "NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotXBotNRT(config_file, config_name, secondary_file));
                else if (type == "ROS_NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRosNRT(config_file, config_name, secondary_file));
       #endif
                else
                        throw std::invalid_argument(error +
                                                    std::string("Unknow robot type"));

        }
#endif

        throw std::invalid_argument(error +
                                    std::string("Unknow middleware ") + config["middleware"].as<std::string>());
}

static std::unique_ptr<mwoibn::robot_class::Robot> create(YAML::Node config, std::string config_name){

        std::string error = std::string("\n robot configuration: \t ") +
                            config_name + std::string("\nerror:\t");

        checkConfig(config, config_name, error);

        std::string type = config["robot"]["layer"].as<std::string>();

 #if ROS
        if (config["middleware"].as<std::string>() == "ROS") {
                if (type == "NRT" || type == "ROS_NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRosNRT(config, config_name));
                else if (type == "EMPTY")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRos(config, config_name));
                else if (type == "RT")
                        throw std::invalid_argument(error +
                                                    std::string("Real time is not supported for ROS"));
                else
                        throw std::invalid_argument(error +
                                                    std::string("Unknow robot type"));
                //return *_robot;
        }
 #endif
 #if XBOT
        if (config["middleware"].as<std::string>() == "XBOT") {
                if (type == "EMPTY")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotXBotFeedback(config, config_name));
                else if (type == "RT")
                        throw std::invalid_argument(error +
                                                    std::string("Real time robot requires access to the shared memory. Please use a RT loader method "));
                //        _robot.reset(new mwoibn::robot_class::RobotXbotRT(config_file, config_name, secondary_file));
       #if ROS
                else if (type == "NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotXBotNRT(config, config_name));
                else if (type == "ROS_NRT")
                        return std::unique_ptr<mwoibn::robot_class::Robot>(new mwoibn::robot_class::RobotRosNRT(config, config_name));
       #endif
                else
                        throw std::invalid_argument(error +
                                                    std::string("Unknow robot type"));

        }
#endif

        throw std::invalid_argument(error +
                                    std::string("Unknow middleware ") + config["middleware"].as<std::string>());
}

protected:
std::unique_ptr<mwoibn::robot_class::Robot> _robot;
//, XBot::SharedMemory::Ptr shared_memory;
};


}
}
#endif
