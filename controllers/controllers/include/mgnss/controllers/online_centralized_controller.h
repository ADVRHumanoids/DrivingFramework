#ifndef __MGNSS_CONTROLLERS_CENTRALIZED_CONTROLLER_H
#define __MGNSS_CONTROLLERS_CENTRALIZED_CONTROLLER_H

#include "mgnss/modules/base.h"

#include <mwoibn/gravity_compensation/simple_qr_gravity_compensation.h>
#include <mwoibn/motor_side_reference/sea_reference.h>
#include <mwoibn/dynamic_models/qr_decomposition.h>


namespace mgnss {
namespace controllers {

class OnlineCentralizedController : public mgnss::modules::Base {

public:
OnlineCentralizedController(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& reference, std::string config_file, std::string name);
OnlineCentralizedController(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& reference, YAML::Node config);

virtual ~OnlineCentralizedController(){
}

virtual void update();
void fullUpdate(const mwoibn::VectorN& command);
void fullUpdate(const mwoibn::VectorN& command, const mwoibn::VectorN& vel_command);

virtual void init();
virtual void send();
virtual void stop(){
}
virtual void close(){
}


virtual void log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);
}

protected:

virtual void _construct(YAML::Node config);
mwoibn::robot_class::Robot& _reference; // this passes current reference

std::unique_ptr<mwoibn::dynamic_models::QrDecomposition> _dynamic_model_ptr;   // online set up
std::unique_ptr<mwoibn::gravity_compensation::SimpleQRGravityCompensation> _gravity_compensation_ptr;
std::unique_ptr<mwoibn::motor_side_reference::SeaReference> _actuation_model_ptr;

bool _motor_side = true;
//  bool _valid = false;

};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
