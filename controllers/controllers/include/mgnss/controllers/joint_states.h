#ifndef __MGNSS__CONTROLLERS_JOINT_STATES_H
#define __MGNSS__CONTROLLERS_JOINT_STATES_H

#include "mgnss/modules/base.h"
#include <mwoibn/point_handling/robot_points_handler.h>
#include "mgnss/higher_level/steering_reference.h"

namespace mgnss
{
namespace controllers
{
class JointStates : public mgnss::modules::Base
{

public:
JointStates(mwoibn::robot_class::Robot& robot);

virtual ~JointStates() {
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
virtual void startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
        logger.start();
}
virtual void log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
        logger.write();
}
protected:
mwoibn::VectorN _position, _velocity, _last_ankle, _last_position, _des_ankle, _init_ankle;
mwoibn::VectorN _pos_ref, _vel_ref;
mwoibn::VectorInt _vel_map, _ankle_map, _vel_sign, _yaw_map;
mwoibn::Vector3 _error, _last;
mwoibn::point_handling::PositionsHandler _wheels;
//  std::vector<mwoibn::Vector3> _wheels_positions;
//mwoibn::robot_class::Robot& _robot;
double _step, _dt, _max_vel = 0.5;
bool _init;



};
}
}

#endif // WHEELED_MOTION_H
