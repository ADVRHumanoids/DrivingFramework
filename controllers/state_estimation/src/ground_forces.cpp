#include "mgnss/state_estimation/ground_forces.h"
#include <iomanip>

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                std::string config_file, std::string name)
        : mgnss::modules::Base(robot), _gravity(robot, {mwoibn::robot_class::DYNAMIC_MODEL::GRAVITY,
                                                        mwoibn::robot_class::DYNAMIC_MODEL::INERTIA,
                                                        mwoibn::robot_class::DYNAMIC_MODEL::NON_LINEAR  }), _points_force(_robot.getDofs())
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
        : mgnss::modules::Base(robot), _gravity(robot, {mwoibn::robot_class::DYNAMIC_MODEL::GRAVITY,
                                                        mwoibn::robot_class::DYNAMIC_MODEL::INERTIA,
                                                        mwoibn::robot_class::DYNAMIC_MODEL::NON_LINEAR}), _points_force(_robot.getDofs())
{
        //_n << 0,0,1;

        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::GroundForces::_initConfig(YAML::Node config){

        _name = config["name"].as<std::string>();
        _filter_torque_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.getDofs(), config["filter"]["torque"]["cut_off_frequency"].as<double>(), config["filter"]["torque"]["damping"].as<double>()));
        _allocate();

        for(auto& contact: _robot.contacts())
          _points_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, *contact));


//        std::vector<std::string> names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};

//        mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> contact_points(_robot.getDofs());

//        mwoibn::VectorInt bi_map = mwoibn::VectorInt::Constant(names.size(), mwoibn::NON_EXISTING);

        // for(auto& contact: _robot.contacts())
        // {
        //     std::string name = _robot.getBodyName(contact->wrench().getBodyId());
        //     if(!std::count(names.begin(), names.end(), name)){
        //       std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
        //       names.erase(std::remove(names.begin(), names.end(), name), names.end());
        //       continue;
        //     }
        //
        //     bi_map[contact_points.size()] = contact_points.size();
        //     _wheels.add(mwoibn::robot_points::TorusModel(
        //                        _robot, mwoibn::point_handling::FramePlus(name,
        //                        _robot.getModel(), _robot.state),
        //                        mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
        //                                     config["reference_axis"][name]["y"].as<double>(),
        //                                     config["reference_axis"][name]["z"].as<double>()),
        //                                     config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
        //                                     contact->getGroundNormal()));
        //
        // }

}
void mgnss::state_estimation::GroundForces::_checkConfig(YAML::Node config){

}

void mgnss::state_estimation::GroundForces::_allocate(){

  _gravity.subscribe({mwoibn::robot_class::DYNAMIC_MODEL::GRAVITY, mwoibn::robot_class::DYNAMIC_MODEL::INERTIA, mwoibn::robot_class::DYNAMIC_MODEL::NON_LINEAR});
  _robot.state.add("OVERALL_FORCE");


  _inertia_inverse.reset(new mwoibn::PseudoInverse(_gravity.getInertia()));
  _contacts_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix(_robot.contacts().jacobianRows(), _robot.contacts().jacobianRows() )));
  _world_contacts.setZero(_robot.contacts().jacobianRows());
  _contacts_jacobian = _robot.contacts().getWorldJacobian();
  _contacts_inversed.setZero(_robot.state.velocity.size(), _robot.state.velocity.size());
  _force_1.setZero(_robot.state.velocity.size());
  _force_2.setZero(_robot.state.velocity.size());
  //world_contact.setZero(12);

  //gf_est.setZero(_robot.contacts().size()*3);
  //_torque_f.setZero(_robot.state.torque.size());

  _filter_torque_ptr->reset(_robot.state.torque.get());

  for(auto& contact: _robot.contacts()){
      //_wheel_center.add(mwoibn::point_handling::FramePlus(contact->wrench().getBodyId(), _robot.getModel(), _robot.state));
      //_velocities.add(mwoibn::point_handling::LinearVelocity(*_wheel_center.end()[-1]));
      _accelerations.add(mwoibn::point_handling::LinearAcceleration(contact->wrench().frame));
    }
  // for(auto& body: _robot.getLinks(mwoibn::eigen_utils::enumerate(actuated)))
  //   _joints.add(mwoibn::robot_points::Joint(body, _robot));

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

      _filter_torque_ptr->update(_robot.state.torque);
      //_robot.updateKinematics();

      // Without explicit acceleration estimation
      _robot.contacts().update(true);
      _gravity.update();
      _contacts_jacobian = _robot.contacts().getWorldJacobian();
      _inertia_inverse->compute(_gravity.getInertia());

      _contacts_inversed = _contacts_jacobian*_inertia_inverse->get();

      _contacts_inverse->compute(_contacts_inversed*_contacts_jacobian.transpose());

      _force_1 = _gravity.getNonlinearEffects() - _robot.state.torque.get();
      _force_2.noalias() = _contacts_inversed*(_force_1);
      _force_1 = _force_2  - _accelerations.getWorld();

      _world_contacts = _contacts_inverse->get()*_force_1;

      for(int i = 0; i < _robot.contacts().size(); i++){
            _robot.contacts()[i].wrench().force.setWorld(_world_contacts.segment<3>(3*i));
            _robot.contacts()[i].wrench().synch();
            //_points_force[i].frame.position.setWorld(_robot.contacts()[i].get());
            // std::cout << "contact: " << _robot.contacts()[i].get().transpose() << std::endl;
      }

      _robot.state["OVERALL_FORCE"].set(-_gravity.getNonlinearEffects() + _robot.state.torque.get() + _robot.contacts().getWorldJacobian().transpose()*_robot.contacts().getReactionForce());
      _points_force.update(true);
      //std::cout << "GF: " << _points_force.getState().transpose() << std::endl;

}

void mgnss::state_estimation::GroundForces::initLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);

        //for(int i = 0; i < 3; i++){
        //   logger.addField(_name + std::string("__com_")+char('x'+i), _com[i]);
        //   logger.addField(_name + std::string("__cop_")+char('x'+i), _cop[i]);
        //   logger.addField(_name + std::string("__s_cop_")+char('x'+i), _f_cop[i]);
        //   logger.addField(_name + std::string("__a_cop_")+char('x'+i), _a_cop[i]);
        //}

        for(int contact = 0; contact < _robot.contacts().size(); contact++){
            for(int i = 0; i < 3; i++){
              logger.addField(_name + std::string("__model_")+std::to_string(contact) + "_" + char('x'+i), _robot.contacts().begin()[contact]->wrench().force.getWorld()[i]);
//              logger.addField(_name + std::string("__residue_")+std::to_string(contact) + "_" + char('x'+i), _world_contacts[contact*3+i]);
              logger.addField(_name + std::string("__point_")+std::to_string(contact) + "_" + char('x'+i), _points_force[contact].get()[i]);
//              logger.addField("force_contact_"+std::to_string(contact) + "_" + char('x'+i), _estimations[contact][i]);
            }
//            logger.addField("wheel_"+std::to_string(contact), _robot.state.acceleration.get(_robot.getDof(_robot.getBodyName(_robot.contacts()[contact].wrench().getBodyId()))[0]));
        }


}
void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);

        //for(int i = 0; i < 3; i++){
        //   logger.addEntry(_name + std::string("__com_")+char('x'+i), _com[i]);
        //   logger.addEntry(_name + std::string("__cop_")+char('x'+i), _cop[i]);
        //   logger.addEntry(_name + std::string("__s_cop_")+char('x'+i), _f_cop[i]);
        //   logger.addEntry(_name + std::string("__a_cop_")+char('x'+i), _a_cop[i]);
        //}

        for(int contact = 0; contact < _robot.contacts().size(); contact++){
            for(int i = 0; i < 3; i++){
              logger.addEntry(_name + std::string("__model_")+std::to_string(contact) + "_" + char('x'+i), _robot.contacts().begin()[contact]->wrench().force.getWorld()[i]);
//              logger.addEntry(_name + std::string("__residue_")+std::to_string(contact) + "_" + char('x'+i), _world_contacts[contact*3+i]);
              logger.addEntry(_name + std::string("__point_")+std::to_string(contact) + "_" + char('x'+i), _points_force[contact].get()[i]);
             // logger.addEntry("force_contact_"+std::to_string(contact) + "_" + char('x'+i), _estimations[contact][i]);
            }
           // logger.addEntry("wheel_"+std::to_string(contact), _robot.state.acceleration.get(_robot.contacts()[contact].wrench().getBodyId()));
        }


}
