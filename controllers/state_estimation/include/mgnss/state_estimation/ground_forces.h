#ifndef __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H
#define __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H

#include <mwoibn/robot_class/robot.h>
#include "mgnss/modules/base.h"
#include <mwoibn/point_handling/robot_points_handler.h>
#include <mwoibn/dynamic_models/basic_model.h>
//#include <mwoibn/filters/iir_second_order.h>
//#include <chrono>

namespace mgnss {

namespace state_estimation {

/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class GroundForces : public mgnss::modules::Base {
public:
GroundForces(mwoibn::robot_class::Robot& robot);
GroundForces(mwoibn::robot_class::Robot& robot, std::string config_file);
GroundForces(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~GroundForces(){
}

virtual void update();
virtual void init();

virtual void send(){
        //_robot.send();
}

virtual void stop(){
}                         // NOT IMPLEMENTED

virtual void close(){
}                          // NOT IMPLEMENTED

virtual void startLog(mwoibn::common::Logger& logger);
virtual void log(mwoibn::common::Logger& logger, double time);

protected:
void _allocate();
void _checkConfig(YAML::Node config);
void _initConfig(YAML::Node config);

mwoibn::dynamic_models::BasicModel _gravity;
mwoibn::VectorBool _selector;

std::unique_ptr<mwoibn::PseudoInverse> _inverse;
mwoibn::Axis _n;
mwoibn::VectorN world_contact;
mwoibn::point_handling::Point::Current angular;
std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_ptr;

mwoibn::Matrix _jac;

mwoibn::VectorN gf_est, f_gf;
std::vector<mwoibn::Matrix> _placements;
mwoibn::Matrix _all;
mwoibn::Vector3 _g, _com, _cop, _f_cop;

};

}
}
#endif
