#include "mgnss/state_estimation/ground_forces.h"
#include <iomanip>

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                std::string config_file, std::string name)
        : mgnss::modules::Base(robot), _gravity(robot)//, _points_force(_robot.getDofs()), _linear_force(_robot.getDofs())
{
        //_n << 0,0,1;

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"][name])
                throw std::invalid_argument(
                              std::string("Couldn't find ground_forces module configuration."));

        config = config["modules"][name];
        config["name"] = name;

        _checkConfig(config);
        _initConfig(config);
}

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                YAML::Node config)
        : mgnss::modules::Base(robot), _gravity(robot)//, _points_force(_robot.getDofs()), _linear_force(_robot.getDofs())
{
        //_n << 0,0,1;

        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::GroundForces::_initConfig(YAML::Node config){

        _name = config["name"].as<std::string>();
        _filter_torque_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.getDofs(), config["filter"]["torque"]["cut_off_frequency"].as<double>(), config["filter"]["torque"]["damping"].as<double>()));
        _allocate();

        // for(auto& contact: _robot.contacts()){
        //   _points_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, *contact));
        // }

        // _linear_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, _robot.centerOfMass()));

}
void mgnss::state_estimation::GroundForces::_checkConfig(YAML::Node config){

}

void mgnss::state_estimation::GroundForces::_allocate(){

  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR});
  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE});

  _robot.state.add("OVERALL_FORCE", _robot.getDofs());
  _robot.state.add("BIAS_FORCE", _robot.getDofs());
  _robot.state.add(_unfiltered_torque, _robot.getDofs());


  //_inertia_inverse.reset(new mwoibn::PseudoInverse(_gravity.getInertia()));
  _contacts_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix(_robot.contacts().jacobianRows(), _robot.contacts().jacobianRows() )));
  _world_contacts.setZero(_robot.contacts().jacobianRows());
  _contacts_jacobian = _robot.contacts().getWorldJacobian();
  _contacts_inversed.setZero(_robot.contacts().jacobianRows(), _robot.state.velocity.size());
  _contacts_temp.setZero(_robot.contacts().jacobianRows(), _robot.contacts().jacobianRows());
  _contacts_transposed = _contacts_jacobian.transpose();

  _force_1.setZero(_robot.state.torque.size());
  _force_2.setZero(_robot.contacts().jacobianRows());
  _force_3.setZero(_robot.contacts().jacobianRows());

  _state.setZero(_robot.state.torque.size());
  _state_no_torque.setZero(_robot.state.torque.size());
  _state_2.setZero(_robot.state.torque.size());
  _set_force.setZero(3);

  _filter_torque_ptr->reset(_robot.state.torque.get());

  for(auto& contact: _robot.contacts()){
      _accelerations.add(mwoibn::point_handling::LinearAcceleration(contact->wrench().frame));
    }

   for(int i = 0; i < 3; i++){
       _log_names.push_back(std::string("__com_")+char('x'+i));
       _log_names.push_back(std::string("__cop_")+char('x'+i));
   }

   for(int contact = 0; contact < _robot.contacts().size(); contact++){
       for(int i = 0; i < 3; i++)
           _log_names.push_back(_name + std::string("__RF_")+std::to_string(contact) + "_" + char('x'+i));
   }

  // for(int i =6; i< _robot.getDofs(); i++)
  //     _log_names.push_back(_name + std::string("__torque_") + _robot.getLinks(i));

}


void mgnss::state_estimation::GroundForces::init(){

        _robot.get();
        _robot.updateKinematics();

        _filter_torque_ptr->computeCoeffs(_robot.rate());

        update();
        _filter_torque_ptr->reset(_robot.state.torque.get());
}

void mgnss::state_estimation::GroundForces::update()
{
      _robot.state[_unfiltered_torque].set(_robot.state.torque);

      _filter_torque_ptr->update(_robot.state.torque);
      //_robot.updateKinematics();

      // Without explicit acceleration estimation
      _robot.contacts().update(true);
      _gravity.update();
      _contacts_jacobian = _robot.contacts().getWorldJacobian();
      _contacts_transposed = _contacts_jacobian.transpose();
      //_inertia_inverse->compute(_gravity.getInertia());

      _contacts_inversed.noalias() = _contacts_jacobian*_gravity.getInertiaInverse();
      _contacts_temp.noalias() = _contacts_inversed*_contacts_transposed;
      _contacts_inverse->compute(_contacts_temp);

      _force_1 = _gravity.getNonlinearEffects() - _robot.state.torque.get();
      _force_2.noalias() = _contacts_inversed*_force_1;
      _force_3 = _force_2  - _accelerations.getWorld(); // ???

      _world_contacts.noalias() = _contacts_inverse->get()*_force_3;

      for(int i = 0; i < _robot.contacts().size(); i++){
            _set_force = _world_contacts.segment<3>(3*i);
            _robot.contacts()[i].wrench().force.setWorld(_set_force);
            _robot.contacts()[i].wrench().synch();
            // std::cout << "i\t" << _robot.contacts()[i].wrench().force.getFixed().transpose() << std::endl;
      }

      _state_no_torque.noalias() = _contacts_transposed*_robot.contacts().getReactionForce();
      // std::cout << "RF\t" << _state_no_torque.transpose() << std::endl;

      _state_no_torque -= _gravity.getNonlinearEffects();

      // std::cout << "nonlinear\t" << _gravity.getNonlinearEffects().transpose() << std::endl;

      _state = _state_no_torque + _robot.state.torque.get();
      _robot.state["OVERALL_FORCE"].set(_state);
      _robot.state["BIAS_FORCE"].set(_state_no_torque);
      // std::cout << __PRETTY_FUNCTION__ << std::string(":\t") << _robot.state["BIAS_FORCE"].get().transpose() << std::endl;
      // std::cout << _robot.state.torque.get().transpose() << std::endl;
      // std::cout << "_world_contacts\t" << _world_contacts.transpose() << std::endl;

      _state_2.noalias() = _gravity.getInertiaInverse()*_state;
      _robot.state.acceleration.set(_state_2);

      // _state = _state_no_torque + _robot.lower_limits.torque.get();
      // _state_2.noalias() = _gravity.getInertiaInverse()*_state;
      // _robot.lower_limits["ACCELERATION"].set(_state_2); // should it be send as state?
      //
      // _state = _state_no_torque + _robot.upper_limits.torque.get();
      // _state_2.noalias() = _gravity.getInertiaInverse()*_state;
      // _robot.upper_limits["ACCELERATION"].set(_state_2);
      // _points_force.update(true);
      //
      // _linear_force.update(true);
      // TEST
      // compute current contact point with respect to the CoM force and torques out of that and see if there is a difference

}

void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);

        _robot.centerOfMass().compute();
        _robot.centerOfPressure().compute();

        int id = 0;
        for(int i = 0; i < 3; i++){
           logger.add(_log_names[id], _robot.centerOfMass().get()[i]);
           logger.add(_log_names[id+1], _robot.centerOfPressure().get()[i]);
            id += 2;
        }

        for(int contact = 0; contact < _robot.contacts().size(); contact++){
            for(int i = 0; i < 3; i++){
              logger.add(_log_names[id], _robot.contacts()[contact].wrench().force.getWorld()[i]);
//              logger.add(_log_names[id+1], _points_force[contact].get()[i]);
              id += 1;
            }
        }

        // for(int i =6; i< _robot.getDofs(); i++){
        //       logger.add(_log_names[id], _robot.state.torque.get()[i]);
        //       id++;
        // }

        // std::cout << "com_inertia matrix\n" << _linear_force.getJacobian() << std::endl;
        // std::cout << "robot mass " << _robot.centerOfMass().mass() << std::endl;
        //std::cout << "inertia inverse\n" << _gravity.getInertiaInverse() << std::endl;
}
