#ifndef __MGNSS__CONTROLLERS__CENTRALIZED_CONTROLLER_H
#define __MGNSS__CONTROLLERS__CENTRALIZED_CONTROLLER_H

#include "mgnss/modules/base.h"

#include <mwoibn/gravity_compensation/simple_qr_gravity_compensation.h>
//#include <mwoibn/motor_side_reference/sea_reference.h>
#include <mwoibn/dynamic_models/qr_decomposition.h>


namespace mgnss {
namespace controllers {

class CentralizedController : public mgnss::modules::Base {

public:
CentralizedController(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
CentralizedController(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~CentralizedController(){
}

virtual void update();
// void fullUpdate(const mwoibn::VectorN& command);
// void fullUpdate(const mwoibn::VectorN& command, const mwoibn::VectorN& vel_command);

virtual void init();
virtual void send();
virtual void stop(){
}
virtual void close(){
}

virtual void log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);
        int counter = 0;

        for(int i = 0; i < 30; i++){
          _log_name = "tau_des_";
          _log_name += std::to_string(i);
          logger.add(_log_name, _robot.command.torque.get()[i]); ++counter;
//          logger.add(_names[counter], _log_command[i]); ++counter;
        }
}

protected:

virtual void _construct(YAML::Node config);
// mwoibn::robot_class::Robot& _reference; // this passes current reference

std::unique_ptr<mwoibn::dynamic_models::QrDecomposition> _dynamic_model_ptr;   // online set up
std::unique_ptr<mwoibn::gravity_compensation::SimpleQRGravityCompensation> _gravity_compensation_ptr;
// std::unique_ptr<mwoibn::motor_side_reference::SeaReference> _actuation_model_ptr;

bool _motor_side = true;
// std::vector<std::string> _names;
std::vector<int> _inactive_dofs;
mwoibn::VectorN _temp_command, _log_command;
std::string _log_name;
//  bool _valid = false;

};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
