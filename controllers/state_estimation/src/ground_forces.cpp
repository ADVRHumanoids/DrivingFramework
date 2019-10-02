#include "mgnss/state_estimation/ground_forces.h"
#include <iomanip>

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                std::string config_file, std::string name)
        : mgnss::modules::Base(robot), _gravity(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file); // Read the configuration file

        // check if the configuration has all required tags
        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"][name])
                throw std::invalid_argument(
                              std::string("Couldn't find ground_forces module configuration."));

        config = config["modules"][name];
        config["name"] = name;

        _initConfig(config);
}

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                YAML::Node config)
        : mgnss::modules::Base(robot), _gravity(robot)
{
        _initConfig(config);
}

void mgnss::state_estimation::GroundForces::_initConfig(YAML::Node config){

        _name = config["name"].as<std::string>();
        // initialize filter using parameters defined in the configuration
        _filter_torque_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.getDofs(), config["filter"]["torque"]["cut_off_frequency"].as<double>(), config["filter"]["torque"]["damping"].as<double>()));
        _allocate();


}


void mgnss::state_estimation::GroundForces::_allocate(){
  _log_name.reserve(1000); // reserve the memmory for the logger identifiers

  /** Declare how many times per loop a specific elements of the dynamic models will be called directly from the dynamic model object
   *  This declaration is used to handle the updates of the dynamic model internally, and to avoid the uneccessary updates of the robot dynamic model.
   */
  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR});
  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE});

  /** Extend the default robot state (that consists of the position, velocity, acceleration and torque) with 
   * two new states: the bias force and the "overall force". The "overall force" adds the measured torque to the bias force.
   * This states are accesible for all other plugins sharing the same robot model.
   */
  _robot.state.add("OVERALL_FORCE", _robot.getDofs());
  _robot.state.add("BIAS_FORCE", _robot.getDofs());


  // Create the object to compute the pseudoinverse
  _contacts_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix(_robot.contacts().jacobianRows(), _robot.contacts().jacobianRows() )));

  // Preallocate the memory for the auxiliary variables
  _world_contacts.setZero(_robot.contacts().jacobianRows());
  _contacts_jacobian = _robot.contacts().getWorldJacobian();
  _contacts_inversed.setZero(_robot.contacts().jacobianRows(), _robot.state.velocity.size());
  _contacts_temp.setZero(_robot.contacts().jacobianRows(), _robot.contacts().jacobianRows());
  _contacts_transposed = _contacts_jacobian.transpose();

  // Preallocate the memory for the auxiliary variables
  _force_1.setZero(_robot.state.torque.size());
  _force_2.setZero(_robot.contacts().jacobianRows());
  _force_3.setZero(_robot.contacts().jacobianRows());

  // Preallocate the memory for the auxiliary variables
  _state.setZero(_robot.state.torque.size());
  _state_no_torque.setZero(_robot.state.torque.size());
  _state_2.setZero(_robot.state.torque.size());
  _set_force.setZero(3);

  // Reset the filter to resize the filter and preallocate the memory
  _filter_torque_ptr->reset(_robot.state.torque.get());

  // Add robot points to compute the \dot J \dot q term. Use the zero interface for the accelerations (\ddot q).
  for(auto& contact: _robot.contacts())
      _accelerations.add(mwoibn::point_handling::LinearAcceleration(contact->wrench().frame, "ZERO"));


}


void mgnss::state_estimation::GroundForces::init(){

  // get the robot feedbacks
        _robot.get();
  // update the kinematics
        _robot.updateKinematics();

  // compute the filter coefficients 
        _filter_torque_ptr->computeCoeffs(_robot.rate());

  // set initial condition (compute for the first time outside the control loop)
        update();
  // Intialize the filter with the measured torques
        _filter_torque_ptr->reset(_robot.state.torque.get());
}

void mgnss::state_estimation::GroundForces::update()
{
      // Filter the measurements

      _filter_torque_ptr->update(_robot.state.torque);

      // update the contact point position and corresponding jacobians
      _robot.contacts().update(true);
      _gravity.update(); // update the dynamic model

      // compute the inverted contact point jacobian
      _contacts_jacobian = _robot.contacts().getWorldJacobian();
      _contacts_transposed = _contacts_jacobian.transpose();

      _contacts_inversed.noalias() = _contacts_jacobian*_gravity.getInertiaInverse();
      _contacts_temp.noalias() = _contacts_inversed*_contacts_transposed;
      _contacts_inverse->compute(_contacts_temp);

      // compute the force acting on the robot
      _force_1 = _gravity.getNonlinearEffects() - _robot.state.torque.get();
      _force_2.noalias() = _contacts_inversed*_force_1;
      _force_2 -= _accelerations.getWorld(); // \dot J \dot q

      /** account for the contact point assumption based on the contact model loaded to the robot modle (e.g. static, rolling)  */
      for(int i = 0; i < _robot.contacts().size(); i++){
          _force_3.segment<3>(3*i) = _robot.contacts()[i].acceleration(); 
      }

      _force_2 += _force_3;
      // compute the estimated ground reaction forces in the world frame
      _world_contacts.noalias() = _contacts_inverse->get()*_force_2;

      // sets the robot contacts based on the estimated state
      for(int i = 0; i < _robot.contacts().size(); i++){
            _set_force = _world_contacts.segment<3>(3*i); // access the part of the estimation for this contact
            _robot.contacts()[i].wrench().force.setWorld(_set_force); // set the force in the robot contact
            _robot.contacts()[i].wrench().synch(); // synchronize the wrench
      }

      // compute the bias force
      _state_no_torque.noalias() = _contacts_transposed*_robot.contacts().getReactionForce();

      _state_no_torque -= _gravity.getNonlinearEffects();

      // compute the overall force
      _state = _state_no_torque + _robot.state.torque.get();
      // set robot states
      _robot.state["OVERALL_FORCE"].set(_state);
      _robot.state["BIAS_FORCE"].set(_state_no_torque);

      // update the center of pressure with the estimated reaction forces
        _robot.centerOfPressure().compute();


}

void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time); // log how much time has passed since the module was turned on

        // log the estimated forces
        for(auto& contact:  _robot.contacts()){
            _log_name = "RF_";
            _log_name += contact->getName();
            _log_name += "_";

            for(int i = 0; i < 3; i++){
              _char = char('x'+i);
              _log_name += _char;
              logger.add(_log_name, contact->wrench().force.getWorld()[i]);
              _log_name.pop_back();
            }
        }

}
