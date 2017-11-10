#ifndef LOADERS_ROBOT_H
#define LOADERS_ROBOT_H

#include <mwoibn/loaders/config.h>

#include <mwoibn/robot_class/robot.h>

#if ROS
  #include <mwoibn/robot_class/robot_ros_nrt.h>
  #if XBOT
    #include <mwoibn/robot_class/robot_xbot_nrt.h>
  #endif
#endif
#if XBOT
  #include <mwoibn/robot_class/robot_xbot_rt.h>
  #include <mwoibn/robot_class/robot_xbot_feedback.h>
#endif


namespace mwoibn
{
namespace loaders
{

class Robot{
public:
  Robot(){}
  ~Robot(){}

  mwoibn::robot_class::Robot& init(std::string config_file, std::string config_name,
                                          std::string secondary_file = ""){
      //load config
      std::cout << "chuj: " << ROS << std::endl;

      YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file, secondary_file);

      std::string error = "config file:\t" + config_file +
                                  std::string("\n robot configuration: \t ") +
                                  config_name + std::string("\nerror:\t");

      // check common elements
      if (!config["middleware"])
        throw std::invalid_argument(error +
            std::string("Please specify a middleware to be loaded."));
      if(!config["robot"])
        throw std::invalid_argument(error +
            std::string("Couldn't find a robot config."));
      if(!config["robot"][config_name])
        throw std::invalid_argument(error +
            std::string("Unknown robot configuration: ") + config_name + std::string("."));
      if(!config["robot"][config_name]["type"])
        throw std::invalid_argument(error +
            std::string("Please specify a robot type."));

      std::string type = config["robot"][config_name]["type"].as<std::string>();


#if ROS
    if (config["middleware"].as<std::string>() == "ROS"){
      if (type == "NRT" || type == "PURE_NRT")
        _robot.reset(new mwoibn::robot_class::RobotRosNRT(config_file, config_name, secondary_file));
      else if (type == "EMPTY")
        _robot.reset(new mwoibn::robot_class::RobotRos(config_file, config_name, secondary_file));
      else if (type == "RT")
        throw std::invalid_argument(error +
            std::string("Real time is not supported for ROS"));
      else
        throw std::invalid_argument(error +
            std::string("Unknow robot type"));
      return *_robot;
    }
#endif
#if XBOT
    if (config["middleware"].as<std::string>() == "XBOT"){
      if (type == "EMPTY")
        _robot.reset(new mwoibn::robot_class::RobotXBotFeedback(config_file, config_name, secondary_file));
      else if (type == "RT")
        throw std::invalid_argument(error +
            std::string("Real time robot requires access to the shared memory. Please use a RT loader method "));
//        _robot.reset(new mwoibn::robot_class::RobotXbotRT(config_file, config_name, secondary_file));
      #if ROS
      else if (type == "NRT")
        _robot.reset(new mwoibn::robot_class::RobotXBotNRT(config_file, config_name, secondary_file));
      else if (type == "PURE_NRT")
        _robot.reset(new mwoibn::robot_class::RobotRosNRT(config_file, config_name, secondary_file));
      #endif
      else
        throw std::invalid_argument(error +
            std::string("Unknow robot type"));

      return *_robot;
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