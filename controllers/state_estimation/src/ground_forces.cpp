#include "mgnss/state_estimation/ground_forces.h"

#include <iomanip>

mgnss::state_estimation::GroundForces::GroundForces(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot), _gravity(robot)//, _ax_oh("ROOT", _robot), _ax_ph("ROOT", _robot)
{
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

        //std::vector<std::string> names = _robot.getLinks(config["chain"].as<std::string>());
        _allocate();

}
void mgnss::state_estimation::GroundForces::_checkConfig(YAML::Node config){

//        if (!config["chain"])
//                throw std::invalid_argument(
//                              std::string("Please specify srdf chain for odometry plugin."));

//        std::cout << "OdometryV2: read " << config["chain"].as<std::string>() << " chain." << std::endl;
//        std::cout << "OdometryV2: filter damping " << config["filter"]["damping"].as<double>() << std::endl;

}

void mgnss::state_estimation::GroundForces::_allocate(){

  vel.setZero(_robot.getDofs());
  world_contact.setZero(12);
  momentum_contact.setZero(12);
  _jac.setZero(4*6, _robot.getDofs());
  _jac_f.setZero(12, _robot.getDofs());

  gravity.setZero(_robot.getDofs());
  nonlinear.setZero(_robot.getDofs());
  acc_f.setZero(_robot.getDofs());
  acc_ft.setZero(_robot.getDofs());
  acc_est.setZero(_robot.getDofs());
  torque.setZero(_robot.getDofs());

  inertia.setZero(_robot.getDofs(), _robot.getDofs());

}


void mgnss::state_estimation::GroundForces::init(){

        _robot.get();
        _robot.updateKinematics();

        update();

}

void mgnss::state_estimation::GroundForces::update()
{
    _robot.get();
    _robot.updateKinematics();

    world_contact <<  _robot.contacts().contact(0).wrench().force.getWorld(),
                      _robot.contacts().contact(1).wrench().force.getWorld(),
                      _robot.contacts().contact(2).wrench().force.getWorld(),
                      _robot.contacts().contact(3).wrench().force.getWorld();


    gravity.noalias() = _gravity.getGravity();

    torque = _robot.state.get(mwoibn::robot_class::INTERFACE::TORQUE);
    mwoibn::PseudoInverse inverser_w(_robot.contacts().getWorldJacobian().transpose());

    inverser_w.compute(_robot.contacts().getWorldJacobian().transpose());
    std::cout << std::fixed;
    std::cout << std::setprecision(10);

    std::cout << "feedback" << world_contact.transpose() << std::endl;

    std::cout << "est_force" << (inverser_w.get()*(gravity-torque)).transpose() << std::endl;

}

void mgnss::state_estimation::GroundForces::startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
        logger.start();
}
void mgnss::state_estimation::GroundForces::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
        logger.write();
}
