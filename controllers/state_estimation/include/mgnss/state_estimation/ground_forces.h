#ifndef __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H
#define __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H

#include <mwoibn/robot_class/robot.h>
#include "mgnss/modules/base.h"
#include <mwoibn/point_handling/robot_points_handler.h>
#include <mwoibn/dynamic_models/basic_model.h>
//#include <mwoibn/robot_points/joint.h>
#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/point_handling/linear_velocity.h"
#include "mwoibn/robot_points/torus_model.h"

#include "mwoibn/point_handling/handler.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/dynamic_points/force.h"
#include "mwoibn/dynamic_points/linear_force.h"

//#include <mwoibn/filters/iir_second_order.h>
//#include <chrono>

namespace mgnss {

namespace state_estimation {

/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class GroundForces : public mgnss::modules::Base {
public:
// GroundForces(mwoibn::robot_class::Robot& robot);
GroundForces(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
GroundForces(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~GroundForces(){
}

virtual void update();
virtual void init();

virtual void send(){
        _robot.send();
}

virtual void stop(){
}
virtual void close(){
}
virtual void log(mwoibn::common::Logger& logger, double time);



protected:
void _allocate();
void _checkConfig(YAML::Node config);
void _initConfig(YAML::Node config);
//void _initCallbacks(YAML::Node config);

std::vector<std::string> _log_names;


mwoibn::point_handling::Handler<mwoibn::point_handling::LinearAcceleration> _accelerations;
mwoibn::dynamic_models::BasicModel _gravity;
// mwoibn::robot_points::Handler<mwoibn::dynamic_points::Force> _points_force;
// mwoibn::robot_points::Handler<mwoibn::dynamic_points::Force> _linear_force;

std::unique_ptr<mwoibn::PseudoInverse> _inertia_inverse, _contacts_inverse;
mwoibn::VectorN _world_contacts, _force_1, _force_2, _force_3, _state;

std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_torque_ptr;

mwoibn::Matrix _contacts_jacobian, _contacts_inversed, _contacts_temp, _contacts_transposed;
mwoibn::Vector3 _des_com;


};

}
}
#endif
