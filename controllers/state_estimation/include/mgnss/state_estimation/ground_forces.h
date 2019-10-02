#ifndef __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H
#define __MGNSS_STATE_ESTIMATION__GROUND_FORCES_H

#include <mwoibn/robot_class/robot.h>
#include "mgnss/modules/base.h"

#include <mwoibn/dynamic_models/basic_model.h>

#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/point_handling/handler.h"


namespace mgnss {

namespace state_estimation {

/**
 * Module to estimate the ground reaction forces from the joint space torque readings based on the whole-body dynamics and assuming known contact point placement.
 * 
 */
class GroundForces : public mgnss::modules::Base {
public:

GroundForces(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
GroundForces(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~GroundForces(){
}

//! Main update function. It is executed at each control step. Computes the estimation.
virtual void update();
//! Init function that sets up the initial conditions.
virtual void init();

//! Share the contact force estimation 
virtual void send(){
        _robot.send();
}

//! Function executed when the module is stopped
virtual void stop(){}
//! Function executed on the module shutdown
virtual void close(){}
//! Function that provdes access to the logger
virtual void log(mwoibn::common::Logger& logger, double time);



protected:
//! Function executed when the module object is created. In contains all memory allocations required for the plugin.
/** This method has to be executed outside the RT loop. It is not real time safe. */ 
void _allocate();
//! Method that checks if all the necessary tags have been defined in the config file. 
/** This method is not used in this plugin. It is defined for the consistency with the interfaces of the other plugins*/
void _checkConfig(YAML::Node config){ }
//! Method called when the module object is created. It calls the methods that use the parameters in the configuration file.
void _initConfig(YAML::Node config);


std::string _log_name, _char; // auxiliary variables to avoid memmory allocation when logging the data in the RT loop

mwoibn::point_handling::Handler<mwoibn::point_handling::LinearAcceleration> _accelerations; // Object that handles multiple objects of the robot_points. Copmutes the term \dot J \dot q in the estimation of the reaction forces.
mwoibn::dynamic_models::BasicModel _gravity; // Dynamic model


std::unique_ptr<mwoibn::PseudoInverse> _contacts_inverse; // Object that computes the pseudo inverse of the contact jacobian

mwoibn::VectorN _world_contacts, _force_1, _force_2, _force_3, _state, _state_2, _state_no_torque; // Auxiliary variables to preallocate memmory for the RT computations with Eigen

std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_torque_ptr; // Instance of the second order infinite respone filter used to filter the measured torques

mwoibn::Matrix _contacts_jacobian, _contacts_inversed, _contacts_temp, _contacts_transposed; // Auxiliary variables to preallocate memmory for the RT computations with Eigen
mwoibn::Vector3 _des_com, _vec_1, _vec_2; // Auxiliary variables to preallocate memmory for the RT computations with Eigen
mwoibn::VectorN _set_force; // Auxiliary variables to preallocate memmory for the RT computations with Eigen

};

}
}
#endif
