#include "mgnss/controllers/compliance_compensation.h"


mgnss::controllers::ComplianceCompensation::ComplianceCompensation(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name) :  modules::Base(robot){

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
        config["name"] = name;
        _construct(config);
}

mgnss::controllers::ComplianceCompensation::ComplianceCompensation(mwoibn::robot_class::Robot& robot, YAML::Node config) :  modules::Base(robot){
        _construct(config);
}

void mgnss::controllers::ComplianceCompensation::_construct(YAML::Node config){

        // if(!config["type"])
        //         throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": configuration doesn't containt required filed 'type'."));
        //
        // if(!config["reference"])
        //         throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": configuration doesn't containt required filed 'reference'."));
        //
        // if(!config["compliance"])
        //         throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": configuration doesn't containt required filed 'compliance'."));

        config["name"].as<std::string>();
        _actuation_model_ptr.reset(new mwoibn::motor_side_reference::SeaCompensation(_robot));

        //_motor_side = config["compliance"].as<bool>();
}

void mgnss::controllers::ComplianceCompensation::init(){
        update();
}

void mgnss::controllers::ComplianceCompensation::send(){
        _robot.send();
}

void mgnss::controllers::ComplianceCompensation::update(){
        //_reference.get();
        //_robot.command.position.set(_reference.state.position.get());
        //_robot.command.velocity.set(_reference.state.velocity.get());
        //_gravity_compensation_ptr->update();
        //if(_motor_side)
        // std::cout << "original\t" << _robot.command.torque.get().transpose() << std::endl;

        _actuation_model_ptr->update();

}
