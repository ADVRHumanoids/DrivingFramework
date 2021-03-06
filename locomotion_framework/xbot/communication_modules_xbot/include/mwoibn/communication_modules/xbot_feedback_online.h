#ifndef COMMUNICATION_MODULES_XBOT_FEEDBACK_ONLINE_H
#define COMMUNICATION_MODULES_XBOT_FEEDBACK_ONLINE_H

#include "mwoibn/common/all.h"
#include "mwoibn/communication_modules/basic_feedback.h"
#include <XBotInterface/RobotInterface.h>

namespace mwoibn
{
namespace communication_modules
{

class XBotFeedbackOnline : public BasicFeedback
{
public:
  XBotFeedbackOnline(mwoibn::robot_class::State& command,
                     mwoibn::robot_class::State& motor_side,
                     mwoibn::robot_class::State& lower_limits,
                     mwoibn::robot_class::State& upper_limits,
                 mwoibn::robot_class::BiMap map, YAML::Node config,
                 XBot::RobotInterface& robot)
      : BasicFeedback(command, map, config), _robot(robot), _lower_limits(lower_limits), _upper_limits(upper_limits), _motor_side(motor_side)
  {
    std::cout << "Loaded direct feedback from the robot - " << config["name"] << std::endl;
        if(!config["limits"])
        throw(std::invalid_argument("Please specify parameter 'limits'"));
        if(!config["limits"]["tolerance"]){
            std::cout << "no limits tolerance given; 0 assumed" << std::endl;
            config["limits"]["tolerance"]["position"] = 0;
            config["limits"]["tolerance"]["velocity"] = 0;
            config["limits"]["tolerance"]["torque"] = 0;
        }
        else{
            if(!config["limits"]["tolerance"]["position"]){
                std::cout << "no limits tolerance for encoder readings given; 0 deg assumed" << std::endl;
                config["limits"]["tolerance"]["position"] = 0;
            }
            else
                std::cout << "Read a encoder readings tolerance as " << config["limits"]["tolerance"]["position"] << " deg." << std::endl;

            if(!config["limits"]["tolerance"]["velocity"]){
                std::cout << "no limits tolerance for velocity feedback given; 0 deg/s assumed" << std::endl;
                config["limits"]["tolerance"]["velocity"] = 0;
            }
            else
                std::cout << "Read a velocity feedback tolerance as " << config["limits"]["tolerance"]["velocity"] << " deg/s." << std::endl;

            if(!config["limits"]["tolerance"]["torque"]){
                std::cout << "no limits tolerance for torque feedback given; 0 Nm assumed" << std::endl;
                config["limits"]["tolerance"]["torque"] = 0;
            }
            else
                std::cout << "Read a torque feedback tolerance as " << config["limits"]["tolerance"]["torque"] << " Nm." << std::endl;
        }
        mwoibn::VectorN lower_original = _lower_limits.position.get();
        mwoibn::VectorN upper_original = _upper_limits.position.get();

        _initLimit(_lower_limits, -config["limits"]["tolerance"]["position"].as<double>()*mwoibn::PI/180.0, "POSITION");
        _initLimit(_lower_limits, -config["limits"]["tolerance"]["velocity"].as<double>()*mwoibn::PI/180.0, "VELOCITY");
        _initLimit(_lower_limits, -config["limits"]["tolerance"]["torque"].as<double>(),                    "TORQUE");
        _initLimit(_upper_limits,  config["limits"]["tolerance"]["position"].as<double>()*mwoibn::PI/180.0, "POSITION");
        _initLimit(_upper_limits,  config["limits"]["tolerance"]["velocity"].as<double>()*mwoibn::PI/180.0, "VELOCITY");
        _initLimit(_upper_limits,  config["limits"]["tolerance"]["torque"].as<double>(),                    "TORQUE");
    /*
        std::cout << "INIT tolerance" << std::endl;
        std::cout << "name\tLL_org\tLL_new\tUP_old\tUP_new" << std::endl;

        for(int i = 0; i < _lower_limits.get("POSITION").size(); i++){
            if (_map.get()[i] == mwoibn::NON_EXISTING)
                std::cout << "unknown\t";
            else
                std::cout << _robot.getJointByDofIndex(_map.get()[i])->getJointName() << "\t";

            std::cout << lower_original[i] << "\t";
            std::cout << _lower_limits.position.get()[i] << "\t";
            std::cout << upper_original[i] << "\t";
            std::cout << _upper_limits.position.get()[i] << std::endl;
        }

    */

    if(_position)
      std::cout << "\tInitialized position interface\n";
    if(_velocity)
      std::cout << "\tInitialized velocity interface\n";
    if(_torque)
      std::cout << "\tInitialized torque interface\n";

    _pub.setZero(_robot.getJointNum());
    std::cout << "\tSuccess" << std::endl;
  }

  virtual ~XBotFeedbackOnline(){}

  bool initialize(){
    _initialized = true;

    if(_position)
      _initialized = _command.position.get().norm() > 1e-12 && _initialized;
    if(_velocity)
      _initialized = _command.velocity.get().norm() > 1e-12 && _initialized;

    if(_torque)
      _initialized = _command.torque.get().norm() > 1e-12 && _initialized;

    return _initialized;
  }

  virtual bool run();

  virtual bool reset();

protected:
  mwoibn::VectorN _pub;
  XBot::RobotInterface& _robot;
  mwoibn::robot_class::State _lower_limits, _upper_limits;
  mwoibn::robot_class::State& _motor_side;

  virtual bool _inLimits(mwoibn::Interface interface);
  virtual bool _inLimits(int i, mwoibn::Interface interface);

  void _initLimit(mwoibn::robot_class::State& limits, double tolerance, mwoibn::Interface interface);
};
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
