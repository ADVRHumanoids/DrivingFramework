#include "mgnss/state_estimation/ground_forces.h"

#include <iomanip>

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot), _gravity(robot)//, _ax_oh("ROOT", _robot), _ax_ph("ROOT", _robot)
{
        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.contacts().size()*3, 1000, 1));

        _n << 0,0,1;
}

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                std::string config_file)
        : mgnss::modules::Base(robot), _gravity(robot)//, _ax_oh("ROOT", _robot), _ax_ph("ROOT", _robot)
{
        _n << 0,0,1;

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"]["ground_forces"])
                throw std::invalid_argument(
                              std::string("Couldn't find ground_forces module configuration."));

        config = config["modules"]["ground_forces"];

        _checkConfig(config);
        _initConfig(config);
}

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot,
                                                YAML::Node config)
        : mgnss::modules::Base(robot), _gravity(robot)//, _ax_oh("ROOT", _robot), _ax_ph("ROOT", _robot)
{
        _n << 0,0,1;

        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::GroundForces::_initConfig(YAML::Node config){

        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.contacts().size()*3, config["filter"]["cut_off_frequency"].as<double>(), config["filter"]["damping"].as<double>()));
        _allocate();

}
void mgnss::state_estimation::GroundForces::_checkConfig(YAML::Node config){

}

void mgnss::state_estimation::GroundForces::_allocate(){

  _inverse.reset(new mwoibn::PseudoInverse(_robot.contacts().getJacobian().transpose()));

  world_contact.setZero(12);

  gf_est.setZero(_robot.contacts().size()*3);
  f_gf.setZero(_robot.contacts().size()*3);


  // _placements.assign(_robot.contacts().size()+1, mwoibn::Matrix::Zero(3,_robot.contacts().size()+1));
  // _all.setZero(3, _robot.contacts().size()+1);
  // _g << 0,0, -9.81;
  // _all.col(_robot.contacts().size()) = _g*_robot.centerOfMass().mass();
}


void mgnss::state_estimation::GroundForces::init(){

        _robot.get();
        _robot.updateKinematics();

        _filter_ptr->computeCoeffs(_robot.rate());

        update();
        _filter_ptr->reset(gf_est);

        _g << 0,0, -9.81;
}

void mgnss::state_estimation::GroundForces::update()
{
    _robot.get();
    _robot.updateKinematics();

    // world_contact <<  _robot.contacts().contact(0).wrench().force.getWorld(),
    //                   _robot.contacts().contact(1).wrench().force.getWorld(),
    //                   _robot.contacts().contact(2).wrench().force.getWorld(),
    //                   _robot.contacts().contact(3).wrench().force.getWorld();

    _inverse->compute(_robot.contacts().getJacobian().transpose());
    gf_est.noalias() = _inverse->get()*(_gravity.getGravity()-_robot.state.torque.get());
    f_gf.noalias() = gf_est;
    _filter_ptr->update(f_gf);

    // _robot.centerOfMass().update(false);
    // _robot.centerOfPressure().update(false);

    // _com = _robot.centerOfMass().get();
    // _cop = _robot.centerOfPressure().get();

//    std::cout << "GF\t" << gf_est.transpose() << std::endl;


//    std::cout << "Simulation" << std::endl;
//    for(auto& contact: _robot.contacts()){
//      std::cout << contact->wrench().getFixed().transpose() << std::endl;
//    }

    for(int i = 0; i < _robot.contacts().size(); i++){
      _robot.contacts().contact(i).wrench().force.setFixed(gf_est.segment<3>(3*i));
      _robot.contacts().contact(i).wrench().torque.setFixed(mwoibn::Vector3(0,0,0));
      _robot.contacts().contact(i).wrench().synch();
    }

//    std::cout << "Estimation" << std::endl;
//    for(auto& contact: _robot.contacts()){
//      std::cout << contact->wrench().getFixed().transpose() << std::endl;
//    }

    _robot.centerOfMass().update(false);
    _robot.centerOfPressure().update(false);

    _com = _robot.centerOfMass().get();
    _cop = _robot.centerOfPressure().get();

    for(int i = 0; i < _robot.contacts().size(); i++){
      _robot.contacts().contact(i).wrench().force.setFixed(f_gf.segment<3>(3*i));
      //_robot.contacts().contact(i).wrench().torque.setFixed(mwoibn::Vector3(0,0,0));
      _robot.contacts().contact(i).wrench().synch();
    }

    _robot.centerOfMass().update(false);
    _robot.centerOfPressure().update(false);

    _f_cop = _robot.centerOfPressure().get();

    // for(auto& placement: _placements)
    //   placement.setZero();
    //
    // mwoibn::Vector3 contact;
    // int size = _robot.contacts().size();
    //
    // for(int i = 0; i < size; i++){
    //   _all.col(i) = gf_est.segment<3>(3*i);
    //   //_all.col(i) = _robot.contacts().contact(i).wrench().force.getWorld();
    //   contact = _robot.contacts().contact(i).getPosition().head<3>();
    //
    //   for(int j = 0; j < _placements.size(); j++){
    //     _placements[j].col(i) += contact;
    //     _placements[i].col(j) -= contact;
    //   }
    // }

     // std::cout << "com\t" << _com.transpose() << std::endl;
     // std::cout << "cop\t" << _cop.transpose() << std::endl;

     //
     // std::cout << "NEW" << std::endl;
     //
     // for(int j = 0; j <  _placements.size(); j++){
     //     _placements[j].col(size) += contact;
     //     _placements[size].col(j) -= contact;
     //
     //  }
     //
     //  int j = 0;
     //
     //  for(auto& placement: _placements){
     //    mwoibn::Vector3 sum = mwoibn::Vector3::Zero() ;
     //    std::cout << std::fixed;
     //    std::cout << std::setprecision(8);
     //    for(int i = 0; i < size+1; i++)
     //      sum += placement.col(i).head<3>().cross(_all.col(i).head<3>());
     //
     //    std::cout << "sum\t" << sum.transpose() << std::endl;
     //
     //    j+=1;
     //  }

}

void mgnss::state_estimation::GroundForces::startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);

        logger.addField("com_x", _com[0]);
        logger.addField("com_y", _com[1]);
        logger.addField("com_z", _com[2]);
        logger.addField("cop_x", _cop[0]);
        logger.addField("cop_y", _cop[1]);
        logger.addField("cop_z", _cop[2]);
        logger.addField("f_cop_x", _f_cop[0]);
        logger.addField("f_cop_y", _f_cop[1]);
        logger.addField("f_cop_z", _f_cop[2]);
        logger.addField("gf_1", gf_est[2]);
        logger.addField("gf_2", gf_est[5]);
        logger.addField("gf_3", gf_est[8]);
        logger.addField("gf_4", gf_est[11]);
        logger.addField("f_gf_1", f_gf[2]);
        logger.addField("f_gf_2", f_gf[5]);
        logger.addField("f_gf_3", f_gf[8]);
        logger.addField("f_gf_4", f_gf[11]);
        logger.start();
}
void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);

        logger.addEntry("com_x", _com[0]);
        logger.addEntry("com_y", _com[1]);
        logger.addEntry("com_z", _com[2]);
        logger.addEntry("cop_x", _cop[0]);
        logger.addEntry("cop_y", _cop[1]);
        logger.addEntry("cop_z", _cop[2]);
        logger.addEntry("f_cop_x", _f_cop[0]);
        logger.addEntry("f_cop_y", _f_cop[1]);
        logger.addEntry("f_cop_z", _f_cop[2]);
        logger.addEntry("gf_1", gf_est[2]);
        logger.addEntry("gf_2", gf_est[5]);
        logger.addEntry("gf_3", gf_est[8]);
        logger.addEntry("gf_4", gf_est[11]);
        logger.addEntry("f_gf_1", f_gf[2]);
        logger.addEntry("f_gf_2", f_gf[5]);
        logger.addEntry("f_gf_3", f_gf[8]);
        logger.addEntry("f_gf_4", f_gf[11]);
        logger.write();
}
