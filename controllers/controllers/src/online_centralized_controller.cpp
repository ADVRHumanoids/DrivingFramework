#include "mgnss/controllers/online_centralized_controller.h"


mgnss::controllers::OnlineCentralizedController::OnlineCentralizedController(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& reference, std::string config_file) :  modules::Base(robot), _reference(reference){

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["wheeled_motion"];
        _construct(config);
}

mgnss::controllers::OnlineCentralizedController::OnlineCentralizedController(mwoibn::robot_class::Robot& robot, mwoibn::robot_class::Robot& reference, YAML::Node config) :  modules::Base(robot), _reference(reference){
        _construct(config);
}

void mgnss::controllers::OnlineCentralizedController::_construct(YAML::Node config){

        if(!config["type"])
                throw std::invalid_argument(std::string("Centralized controller: configuration doesn't containt required filed 'type'."));

        if(!config["reference"])
                throw std::invalid_argument(std::string("Centralized controller: configuration doesn't containt required filed 'reference'."));

        if(!config["compliance"])
                throw std::invalid_argument(std::string("Centralized controller: configuration doesn't containt required filed 'compliance'."));

        if(config["type"].as<std::string>() == "online")
                _dynamic_model_ptr.reset(
                        new mwoibn::dynamic_models::QrDecomposition(_robot));
        else if(config["type"].as<std::string>() == "offline")
                _dynamic_model_ptr.reset(
                        new mwoibn::dynamic_models::QrDecomposition(_reference));
        else
                throw std::invalid_argument(std::string("Centralized controller: unknown type '") +  config["type"].as<std::string>() + std::string("', avilable option 'online' and 'offline'. "));

        _gravity_compensation_ptr.reset(
                new mwoibn::gravity_compensation::SimpleQRGravityCompensation(
                        *_dynamic_model_ptr, _robot));

        _actuation_model_ptr.reset(new mwoibn::motor_side_reference::SeaReference(
                                           _robot, *_gravity_compensation_ptr));

        _motor_side = config["compliance"].as<bool>();
}

void mgnss::controllers::OnlineCentralizedController::init(){
        update();
}

void mgnss::controllers::OnlineCentralizedController::send(){
        _robot.send();
}

void mgnss::controllers::OnlineCentralizedController::update(){
        _reference.get();
        _robot.command.set(_reference.state.get(),
                           mwoibn::robot_class::INTERFACE::POSITION);
        _robot.command.set(_reference.state.get(mwoibn::robot_class::INTERFACE::VELOCITY),
                                              mwoibn::robot_class::INTERFACE::VELOCITY);
        _gravity_compensation_ptr->update();
        if(_motor_side)
                _actuation_model_ptr->update();

}


void mgnss::controllers::OnlineCentralizedController::fullUpdate(const mwoibn::VectorN& reference){

        if(!_robot.get()) return;

        _robot.updateKinematics();

        _robot.command.set(reference,
                           mwoibn::robot_class::INTERFACE::POSITION);

        update();

        _robot.send();
        _robot.wait();
}

void mgnss::controllers::OnlineCentralizedController::fullUpdate(const mwoibn::VectorN& reference, const mwoibn::VectorN& vel_reference){

        if(!_robot.get()) return;

        _robot.updateKinematics();

        _robot.command.set(reference,
                           mwoibn::robot_class::INTERFACE::POSITION);

        _robot.command.set(vel_reference,
                           mwoibn::robot_class::INTERFACE::VELOCITY);

        update();

        _robot.send();
        _robot.wait();
}
