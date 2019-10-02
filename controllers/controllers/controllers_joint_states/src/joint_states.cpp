#include "mgnss/controllers/joint_states.h"

mgnss::controllers::JointStates::JointStates(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name): modules::Base(robot){
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][name];
  config["name"] = name;
  _allocate(config);

}
mgnss::controllers::JointStates::JointStates(mwoibn::robot_class::Robot& robot, YAML::Node config): modules::Base(robot){
  //config["name"] = name;
  _allocate(config);

}

void mgnss::controllers::JointStates::_allocate(YAML::Node config){


   if(!config["name"])
        throw(std::invalid_argument("JointStates: Unknown controller name"));
    
  _name = config["name"].as<std::string>();
        
   mwoibn::VectorInt temp_ = _robot.getDof(_robot.getLinks(config["chain"].as<std::string>()));
  _pos_map.setConstant(_robot.getDofs(), false);
  for(int i = 0; i < temp_.size(); i++)
    _pos_map[temp_[i]] = true;

  _vel_map = _robot.selectors().get("wheels").which();
  //_ankle_map = _robot.selectors().get("camber").which();
  //_yaw_map = _robot.selectors().get("yaws").which();

  _velocity.setZero(_vel_map.size());
//  _vel_sign.setOnes(_vel_map.size());
  // _last_ankle.setZero(_ankle_map.size());
//  _des_ankle.setZero(_ankle_map.size());

  _position = _robot.state.position.get();
  _last_position = _position;

  _pos_ref = _position;
  _vel_ref = _velocity;
//  _init_ankle.setZero(_ankle_map.size());

}

void mgnss::controllers::JointStates::init(){

        _position.noalias() = _robot.state.position.get();

//        _robot.state.position.get(_des_ankle, _ankle_map);
//        _robot.state.position.get(_init_ankle, _ankle_map);

        _pos_ref.noalias() = _position;
        _vel_ref.noalias() = _velocity;

        // _robot.state.position.get(_last_ankle, _ankle_map);
        _robot.command.position.set(_position);

}

bool mgnss::controllers::JointStates::setFullPosition(std::string name)
{
        if (!_robot.groupStates().isDefined(name)) return false;

        for(int i = 0; i < _pos_ref.size(); i++) {
                if(_robot.groupStates().get(name).get()[i] != mwoibn::NON_EXISTING)
                        _pos_ref[i] = _robot.groupStates().get(name).get()[i];
        }

        _init = false;
        return true;

}


void mgnss::controllers::JointStates::update()
{
        // _robot.state.position.get(_last_ankle, _ankle_map);

        _last_position.noalias() = _position;

        for (int i = 0; i < _position.size(); i++)
        {
                if(_pos_ref[i] == mwoibn::NON_EXISTING) continue;
                if (std::fabs(_pos_ref[i] - _position[i]) > _step)
                {
                        if (_pos_ref[i] - _position[i] > 0)
                                _position[i] += _step;
                        else
                                _position[i] -= _step;
                }
                else
                        _position[i] = _pos_ref[i];
        }

}

void mgnss::controllers::JointStates::send()
{
        _robot.command.position.set(_position, _pos_map);
        _robot.command.velocity.set(_velocity, _vel_map);
        _robot.send();

        // _robot.update();
}


bool mgnss::controllers::JointStates::setVelocity(std::string name, double vel){

        double dof = mwoibn::NON_EXISTING;
        try{
                if(_robot.getDof(name).size()) {
                        dof = _robot.getDof(name)[0];
                }
                else
                        return false;
        }
        catch (const std::out_of_range& e) {
                return false;
        }
        for(int i = 0; i < _vel_map.size(); i++) {
                if (_vel_map[i] == dof) {
                        _velocity[i] = vel;
                        return true;
                }
        }

        return false;
}

bool mgnss::controllers::JointStates::setPosition(std::string name, double pos){

        double dof;
        try{

                if(_robot.getDof(name).size()) {

                        dof = _robot.getDof(name)[0];
                }
                else{
                        return false;
                }
        }
        catch (const std::out_of_range& e) {

                return false;
        }

        _pos_ref[dof] = pos;
        return true;
}
