#ifndef __MGNSS__CONTROLLERS_JOINT_STATES_H
#define __MGNSS__CONTROLLERS_JOINT_STATES_H

#include "mgnss/modules/base.h"

namespace mgnss
{
namespace controllers
{
class JointStates : public mgnss::modules::Base
{

public:
JointStates(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
JointStates(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~JointStates() {
}

bool reset(){
        _pos_ref = _robot.state.position.get();
        _position = _robot.state.position.get();
        return true;
}
bool setVelocity(std::string name, double vel);
bool setPosition(std::string name, double pos);

//  bool setPosition(std::string name);
bool setFullPosition(std::string name);

void step(double step) {
        _step = std::min(step, _max_vel*_robot.rate());
}
virtual void init();
virtual void update();
virtual void send();
virtual void stop(){
}
virtual void close(){
}


virtual void log(mwoibn::common::Logger& logger, double time){
        logger.add("time", time);
}
protected:
mwoibn::VectorN _position, _velocity, _last_position;//, _des_ankle;//, _init_ankle;
mwoibn::VectorN _pos_ref, _vel_ref;
mwoibn::VectorInt _vel_map;//, _vel_sign;//, _yaw_map;
mwoibn::VectorBool _pos_map;
mwoibn::Vector3 _error, _last;
//  std::vector<mwoibn::Vector3> _wheels_positions;
//mwoibn::robot_class::Robot& _robot;
double _step, _dt, _max_vel = 2.0;
bool _init;

void _allocate(YAML::Node config);

};
}
}

#endif // WHEELED_MOTION_H
