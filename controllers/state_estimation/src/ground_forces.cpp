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
        _base_ptr.reset(new mwoibn::filters::IirSecondOrder(6, config["filter"]["torque"]["cut_off_frequency"].as<double>(), 3));
        _allocate();

        // for(auto& contact: _robot.contacts()){
        //   _points_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, *contact));
        // }

        // _linear_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, _robot.centerOfMass()));

}
void mgnss::state_estimation::GroundForces::_checkConfig(YAML::Node config){

}

void mgnss::state_estimation::GroundForces::_allocate(){
  _log_name.reserve(1000);

  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR});

  _robot.state.add("OVERALL_FORCE", _robot.getDofs());
  _robot.state.add("BIAS_FORCE", _robot.getDofs());
  _robot.state.add("FEEDBACK_FORCE", _robot.getDofs());

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
  _base_temp.setZero(6);
  _base_ptr->reset(_base_temp);
  std::cout << "reset" << std::endl;

  for(auto& contact: _robot.contacts()){
      _accelerations.add(mwoibn::point_handling::LinearAcceleration(contact->wrench().frame, "ZERO"));

      std::string name = _robot.getBodyName(contact->wrench().getBodyId());

      _wheel_frames.add(mwoibn::point_handling::FramePlus( name, _robot.getModel(), _robot.state));
      _wheel_centers.add(mwoibn::point_handling::AngularVelocity( _wheel_frames.end(0) ));// ??

    }
    _base_temp.setZero(6);
    _base_frame.add(mwoibn::point_handling::FramePlus( "pelvis", _robot.getModel(), _robot.state));
    _base_wrench.add(mwoibn::point_handling::Wrench(_base_frame[0]));
    _base_velocity.add(mwoibn::point_handling::SpatialVelocity(_base_frame[0]));
}


void mgnss::state_estimation::GroundForces::init(){

        _robot.get();
        _robot.updateKinematics();

        _filter_torque_ptr->computeCoeffs(_robot.rate());
        _base_ptr->computeCoeffs(_robot.rate());
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

      _contacts_inversed.noalias() = _contacts_jacobian*_gravity.getInertiaInverse();
      _contacts_temp.noalias() = _contacts_inversed*_contacts_transposed;
      _contacts_inverse->compute(_contacts_temp);

      _force_1 = _gravity.getNonlinearEffects() - _robot.state.torque.get(); // with  zero interface for accelerations it wont - the RBDL model is not updated anyway?
      _force_2.noalias() = _contacts_inversed*_force_1;
      _force_2 -= _accelerations.getWorld();

      for(int i = 0; i < _robot.contacts().size(); i++){
          // _force_3.segment<3>(3*i) = _wheel_frames[i].rotation().getWorld()*_robot.contacts()[i].getFrame().getLinearFixed(); // what about the contact assumptions here?
          //_vec_2 = _wheel_centers[i].getWorld().head<3>().cross(_vec_1); // Add assummed acceleration
          _force_3.segment<3>(3*i) = _robot.contacts()[i].acceleration();
      }

      _force_2 += _force_3;
      _world_contacts.noalias() = _contacts_inverse->get()*_force_2;

      for(int i = 0; i < _robot.contacts().size(); i++){
            _set_force = _world_contacts.segment<3>(3*i);
            _robot.contacts()[i].wrench().force.setWorld(_set_force);
            _robot.contacts()[i].wrench().synch();
      }

      _state_no_torque.noalias() = _contacts_transposed*_robot.contacts().getReactionForce();
      _state_no_torque -= _gravity.getNonlinearEffects();
      _state = _state_no_torque + _robot.state.torque.get();
      _robot.state["OVERALL_FORCE"].set(_state); // ACTING FORCE?
      _robot.state["BIAS_FORCE"].set(_state_no_torque);
      _base_ptr->update(_base_temp);

      // _robot.state["OVERALL_FORCE"].set(_base_temp, _base_ids);

      std::cout << "est_force\t" << _state.head<6>().transpose() << std::endl;
      // ACCELERATION ESTIMATION
      //_state_2.noalias() = _gravity.getInertiaInverse()*_state;
      //_robot.state.acceleration.set(_state_2);


}

void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);

        _robot.centerOfMass().compute();
        _robot.centerOfPressure().compute();

        for(int i = 0; i < 3; i++){
          _log_name = "com_";
          _char = char('x'+i);
          _log_name += _char;
           logger.add(_log_name, _robot.centerOfMass().get()[i]);
           _log_name = "cop_";
           _log_name += _char;
           logger.add(_log_name, _robot.centerOfPressure().get()[i]);
        }




        for(int contact = 0; contact < _robot.contacts().size(); contact++){
            _log_name = "RF_";
            //_log_name += std::to_string(contact);
            _log_name += _robot.contacts()[contact].getName();
            _log_name += "_";

            for(int i = 0; i < 3; i++){
              _char = char('x'+i);
              _log_name += _char;
              logger.add(_log_name, _robot.contacts()[contact].wrench().force.getWorld()[i]);
              _log_name.pop_back();
              // logger.add(_log_names[id], _force_3[3*contact+i]); ++id;
            }
        }

        for(int i =0; i< 6; i++){
              _char = char('x'+i);

              _log_name = "flitered_F_";
              _log_name += _char;
               logger.add(_log_name, _robot.state["OVERALL_FORCE"].get()[i]);

               _log_name = "est_F_";
               _log_name += _char;
               logger.add(_log_name, _robot.state["FEEDBACK_FORCE"].get()[i]);
        }

        // std::cout << "com_inertia matrix\n" << _linear_force.getJacobian() << std::endl;
        // std::cout << "robot mass " << _robot.centerOfMass().mass() << std::endl;
        //std::cout << "inertia inverse\n" << _gravity.getInertiaInverse() << std::endl;
}
